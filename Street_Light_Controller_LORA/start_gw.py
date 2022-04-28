
import json
import os

gateway_conf_filename = "gateway_conf.json"

gateway_json_array={}

#recover each field different than 0 and call post_processing_gw.py with parameters
call_string_cpp = "sudo ./lora_gateway"
call_string_python = " | python post_processing_gw.py"
call_string_log_gw = ""

def start_config_from_json() :
	#open json file to recover values
	f = open(os.path.expanduser(gateway_conf_filename),"r")
	lines = f.readlines()
	f.close()
	array = ""
	#get all the lines in a string
	for line in lines :
		array += line

	global gateway_json_array
	#change it into a python array
	gateway_json_array = json.loads(array)

	global call_string_cpp
	global call_string_python
	global call_string_log_gw

	try:
		if gateway_json_array["gateway_conf"]["raw"] :
			call_string_cpp += " --raw"
	except KeyError:
		pass

	try:
		if gateway_json_array["gateway_conf"]["log_post_processing"] :
			call_string_log_gw = " | python log_gw.py"
	except KeyError:
		pass

	try:
		lora_mode = gateway_json_array["radio_conf"]["mode"]
	except KeyError:
		lora_mode = 1
		
	#LoRa mode are now handled by this script and translated into BW, CR and SF combination
	#LoRa mode 11 still indicated LoRaWAN mode (BW125) in sub-GHz band, SF can be separately defined then			
	if lora_mode != -1:
		if lora_mode == 1:
			call_string_cpp += " --bw 125 --cr 5 --sf 12"
		elif lora_mode == 2:
			call_string_cpp += " --bw 250 --cr 5 --sf 12"
		elif lora_mode == 3:
			call_string_cpp += " --bw 125 --cr 5 --sf 10"
		elif lora_mode == 4:
			call_string_cpp += " --bw 500 --cr 5 --sf 12"
		elif lora_mode == 5:
			call_string_cpp += " --bw 250 --cr 5 --sf 10"
		elif lora_mode == 6:
			call_string_cpp += " --bw 500 --cr 5 --sf 11"
		elif lora_mode == 7:
			call_string_cpp += " --bw 250 --cr 5 --sf 9"
		elif lora_mode == 8:
			call_string_cpp += " --bw 500 --cr 5 --sf 9"
		elif lora_mode == 9:
			call_string_cpp += " --bw 500 --cr 5 --sf 8"
		elif lora_mode == 10:
			call_string_cpp += " --bw 500 --cr 5 --sf 7"
		elif lora_mode == 11:
			call_string_cpp += " --mode 11 --bw 125 --cr 5 --sf %s" % (str(gateway_json_array["radio_conf"]["sf"]))
	else:
		call_string_cpp += " --bw %s --cr %s --sf %s" % (str(gateway_json_array["radio_conf"]["bw"]),str(gateway_json_array["radio_conf"]["cr"]),str(gateway_json_array["radio_conf"]["sf"]))
		
	try:
		if gateway_json_array["radio_conf"]["ch"] != -1 :
			call_string_cpp += " --ch %s" % str(gateway_json_array["radio_conf"]["ch"])
		elif gateway_json_array["radio_conf"]["freq"] != -1:
			call_string_cpp += " --freq %s" % str(gateway_json_array["radio_conf"]["freq"])
	except KeyError:
		pass

	try:
		if gateway_json_array["gateway_conf"]["downlink"]==-1 :
			call_string_cpp += " --ndl"
	except KeyError:
		pass

	print call_string_cpp+call_string_python+call_string_log_gw
	#launch the commands
	os.system(call_string_cpp + call_string_python + call_string_log_gw)

if __name__ == "__main__":
	start_config_from_json()
