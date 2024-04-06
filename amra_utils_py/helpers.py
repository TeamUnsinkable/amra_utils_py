import os
import yaml

def process_yaml_input(yaml_data_path, config_key):
    buttons = []
    axes = []
    #print(os.getcwd())
    #print(os.path.join(os.getcwd(), yaml_data_path))
    #TODO: Fix path completion
    with open(os.path.join(os.getcwd(), yaml_data_path), 'r') as book:
        yaml_data = book.read()
   
    # Parse as yaml data
    yaml_data = yaml.safe_load(yaml_data)
    entry = yaml_data[config_key]
    if entry == '':
        return
    # Correct JSON entry at id
    for config in entry['mapping']:
        parameter = {"index": config['index'], "dest_topic": config['dest_topic']}
        if config['type'] == 0:
            parameter['joystick_topic'] = "{}/btn{}".format(entry['topic_base'], config["index"]) 
            buttons.append(parameter)
        elif config['type'] == 1:
            parameter['joystick_topic'] = "{}/axs{}".format(entry['topic_base'], config["index"])
            axes.append(parameter)
    return buttons, axes
