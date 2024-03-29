import urllib2
import subprocess
import time
import ssl
import socket
import datetime
import sys
import re

#don't generate pyc (no compilation of imported module) so change in key_* file can be done dynamically
sys.dont_write_bytecode = True

# get key definition from external file to ease
# update of cloud script in the future
import key_ThingSpeak

try:
	key_ThingSpeak.source_list
except AttributeError:
	key_ThingSpeak.source_list=[]

try:
	key_ThingSpeak.field_association
except AttributeError:
	key_ThingSpeak.field_association=[]

try:
	key_ThingSpeak.nomenclature_association
except AttributeError:
	key_ThingSpeak.nomenclature_association=[]

try:
	key_ThingSpeak.key_association
except AttributeError:
	key_ThingSpeak.key_association=[]
			
# didn't get a response from thingspeak server?
connection_failure = False

# retry if return from server is 0?
retry = False

# function to check connection availability with the server
def test_network_available():
	connection = False
	iteration = 0
	response = None
	
	# we try 4 times to connect to the server.
	while(not connection and iteration < 4):
		try:
			# 3sec timeout in case of server available but overcrowded
			response=urllib2.urlopen('https://api.thingspeak.com/', timeout=3)
			connection = True
		except urllib2.URLError, e: pass
		except socket.timeout: pass
		except ssl.SSLError: pass
	    	
		# if connection_failure == True and the connection with the server is unavailable, don't waste more time, exit directly
		if (connection_failure and response is None):
			print('Thingspeak: the server is still unavailable')
			iteration = 4
	    # print connection failure
		elif (response is None):
			print('Thingspeak: server unavailable, retrying to connect soon...')
			# wait before retrying
			time.sleep(1)
			iteration += 1

	return connection
	
#upload multiple data
#this is the only way to upload data 
#format of data is now TC/22.5/HU/24/LUM/345...

def thingspeak_uploadMultipleData(data, src, nomenclatures):
	global connection_failure
	
	connected = test_network_available()
	
	# if we got a response from the server, send the data to it
	if(connected):
		connection_failure = False
		
		print("ThingSpeak: uploading (multiple)")
		print 'rcv msg to log (\!) on ThingSpeak (',			

		found_key_association=False

		# check if we have a key_association field
		# priority is given to key association defined on gateway
		if (len(key_ThingSpeak.key_association)!=0):
			for k in range(0, len(key_ThingSpeak.key_association)):
				# found a key for this source node
				if src in key_ThingSpeak.key_association[k][1:]:
					found_key_association=True
					data[0]=key_ThingSpeak.key_association[k][0]
					print data[0],

		if (len(key_ThingSpeak.key_association)==0 or found_key_association==False):
			#use default channel?
			if data[0]=='':
				data[0]=key_ThingSpeak._def_thingspeak_channel_key
				print 'default',
			else:	
				print data[0],
			
		print ',',

		if (len(key_ThingSpeak.field_association)==0):
			defined_field_association=False
		else:
			defined_field_association=True

		if (len(key_ThingSpeak.nomenclature_association)==0):
			defined_nomenclature_association=False
		else:
			defined_nomenclature_association=True
			
		found_field_association=False
			
		#do we have key_ThingSpeak.field_association defined?
		#again, priority is given to field association defined on gateway
		if defined_field_association==True:
			i = 0
			while i < len(key_ThingSpeak.field_association):
				if key_ThingSpeak.field_association[i][0]==src:
					#found a field for that sensor
					found_field_association=True
					fieldNumber=key_ThingSpeak.field_association[i][1]
					print fieldNumber,
				i = i+1
					
		if (defined_field_association==False) or (found_field_association==False):
			#use default field?
			if data[1]=='':
				fieldNumber = 1
				print 'default',
			else:
				fieldNumber = int(data[1])
				print data[1],				
	
		print '): '
		
		if found_field_association==True:
			print 'field index set by field association for sensor '+str(src)
		
		i=0

		cmd = 'curl -s -k -X POST --data '
								
		while i < len(data)-2:
					
			#use nomemclature association
			field_offset=0
			the_fieldNumber=fieldNumber+i
			
			if (defined_nomenclature_association):
				for item in key_ThingSpeak.nomenclature_association:
					if nomenclatures[i]==item[0]:
						print 'found a nomenclature association for '+nomenclatures[i]
						field_offset=item[1]
						print 'will use field offet of '+str(field_offset)
						the_fieldNumber=fieldNumber+field_offset
						print 'field index will be '+str(the_fieldNumber)
					
			if (i==0):
				cmd += 'field'+str(the_fieldNumber)+'='+data[i+2]
			else:
				cmd += '&field'+str(the_fieldNumber)+'='+data[i+2]
				
			i += 1
			
		cmd += ' https://api.thingspeak.com/update?key='+data[0]
		
		print("ThingSpeak: will issue curl cmd")
		print(cmd)
		args = cmd.split()
	
		#retry enabled
		if (retry) :
			out = '0'
			iteration = 0
		
			while(out == '0' and iteration < 6 and not connection_failure) :
				try:
					out = subprocess.check_output(args, shell=False)

					#if server return 0, we didn't wait 15sec, wait then
					if(out == '0'):
						print('ThingSpeak: less than 15sec between posts, retrying in 3sec')
						iteration += 1
						time.sleep( 3 )
					else:
						print('ThingSpeak: returned code from server is %s' % out)
				except subprocess.CalledProcessError:
					print("ThingSpeak: curl command failed (maybe a disconnection)")
					
					#update connection_failure
					connection_failure = True
				
		#retry disabled
		else :
			try:
				out = subprocess.check_output(args, shell=False)
				if (out == '0'):
					print('ThingSpeak: returned code from server is %s, do not retry' % out)
				else :
					print('ThingSpeak: returned code from server is %s' % out)
				
			except subprocess.CalledProcessError:
				print("ThingSpeak: curl command failed (maybe a disconnection)")
				connection_failure = True
	else:
		connection_failure = True
		
	if(connection_failure):
		print("ThingSpeak: not uploading")
			
	
def thingspeak_setRetry(retry_bool):

	global retry
	retry = retry_bool
	
# main
# -------------------

def main(ldata, pdata, rdata, tdata, gwid):

	#this is common code to process packet information provided
	#by the main gateway script (i.e. post_processing_gw.py)
	#these information are provided in case you need them	
	arr = map(int,pdata.split(','))
	dst=arr[0]
	ptype=arr[1]				
	src=arr[2]
	seq=arr[3]
	datalen=arr[4]
	SNR=arr[5]
	RSSI=arr[6]
	
	#LoRaWAN packet
	if dst==256:
		src_str="0x%0.8X" % src
	else:
		src_str=str(src)	

	if (src_str in key_ThingSpeak.source_list) or (len(key_ThingSpeak.source_list)==0):

		#remove any space in the message as we use '/' as the delimiter
		#any space characters may introduce error
		ldata=ldata.replace(' ', '')
				
		#this part depends on the syntax used by the end-device
		#we use: thingspeak_channel#thingspeak_field#TC/22.4/HU/85... 
		#ex: ##TC/22.4/HU/85... or TC/22.4/HU/85... or thingspeak_channel##TC/22.4/HU/85... 
		#or #thingspeak_field#TC/22.4/HU/85... to use some default value
				
		# get number of '#' separator
		nsharp = ldata.count('#')	
				
		#no separator
		if nsharp==0:
			#will use default channel and field
			data=['','']
		
			#contains ['', '', "s1", s1value, "s2", s2value, ...]
			data_array = data + re.split("/", ldata)		
		elif nsharp==1:
			#only 1 separator
		
			data_array = re.split("#|/", ldata)
		
			#if the first item has length > 1 then we assume that it is a channel write key
			if len(data_array[0])>1:
				#insert '' to indicate default field
				data_array.insert(1,'');		
			else:
				#insert '' to indicate default channel
				data_array.insert(0,'');		
		else:
			#contains [channel, field, "s1", s1value, "s2", s2value, ...]
			data_array = re.split("#|/", ldata)	
		
		#just in case we have an ending CR or 0
		data_array[len(data_array)-1] = data_array[len(data_array)-1].replace('\n', '')
		data_array[len(data_array)-1] = data_array[len(data_array)-1].replace('\0', '')	
	
		#test if there are characters at the end of each value, then delete these characters
		i = 3
		while i < len(data_array) :
			while not data_array[i][len(data_array[i])-1].isdigit() :
				data_array[i] = data_array[i][:-1]
			i += 2
		
		#get number of '/' separator
		nslash = ldata.count('/')
	
		nomenclatures = []	
		#data to send to thingspeak
		data = []
		data.append(data_array[0]) #channel (if '' default)
		data.append(data_array[1]) #field (if '' default)		

		if nslash==0:
			# old syntax without nomenclature key, so insert only one key
			# we use DEF
			nomenclatures.append("DEF")
			data.append(data_array[2])
		else:
			# completing nomenclatures and data
			i=2
			while i < len(data_array)-1 :
				nomenclatures.append(data_array[i])
				data.append(data_array[i+1])
				i += 2
					
		thingspeak_uploadMultipleData(data, src, nomenclatures)
	else:
		print "Source is not is source list, not sending with CloudThingSpeak.py"				
	
if __name__ == "__main__":
	main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
	
