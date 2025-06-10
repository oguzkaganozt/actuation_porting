import asyncio
import time
import websockets
#from pprint import pprint # Optional for testing and development purposes.
import avh_api_async as AvhAPI
from avh_api_async.rest import ApiException as AvhAPIException
from dotenv import load_dotenv
import os
import sys

# load the api token from the .env file
load_dotenv()
apiEndpoint = os.getenv('AVH_API_ENDPOINT')
apiToken = os.getenv('AVH_API_TOKEN')
instanceFlavor = os.getenv('AVH_INSTANCE_FLAVOR')
instanceName = os.getenv('AVH_INSTANCE_NAME')

# check if the api token and endpoint are set
if not apiToken or not apiEndpoint:
  print('AVH_API_TOKEN or AVH_API_ENDPOINT is not set in the .env file')
  exit(1)
  
async def main():
  configuration = AvhAPI.Configuration(host = apiEndpoint)
  
  async with AvhAPI.ApiClient(configuration=configuration) as api_client:
    api_instance = AvhAPI.ArmApi(api_client)
  
    # Log in
    print('Logging in ...')
    try:
        token_response = await api_instance.v1_auth_login({
          "apiToken": apiToken
        })
        configuration.access_token = token_response.token
    except AvhAPIException as e:
        print('Exception when calling v1_auth_login: %s\n' % e)
        exit(1)
    print('Logged in.')
    
    # Get the project
    print('Finding the project ...')
    api_response = await api_instance.v1_get_projects()
    projectId = api_response[0].id
    print('Found project ' + projectId + '.')

    # Get the instance by name
    print('Finding the instance ...')
    try:
        api_response = await api_instance.v1_get_instances(
          name=instanceName
        )
        print(f'API response received: {len(api_response)} instances found')
        if len(api_response) == 0:
            print(f'No instance found with name: {instanceName}')
            exit(1)
        instanceId = api_response[0].id
        serviceIp = api_response[0].service_ip
        flavor = api_response[0].flavor
        type = api_response[0].type
        state = api_response[0].state
    except AvhAPIException as e:
        print('Exception when calling v1_get_instances: %s\n' % e)
        exit(1)
    except Exception as e:
        print(f'Unexpected error when finding instance: {e}')
        exit(1)

    print('--------------------------------')
    print('Found instance ' + instanceId + '.')
    print('Name: ' + instanceName)
    print('Service IP: ' + serviceIp)
    print('Flavor: ' + flavor)
    print('Type: ' + type)
    print('State: ' + state)
    print('--------------------------------')
  
    # Upload the custom firmware package
    print('Uploading the custom firmware package ...')
    try:
      api_response = await api_instance.v1_create_image('fwbinary', 'plain', 
        name="zephyr.elf",
        instance=instanceId,
        file=os.path.join(sys.path[0], 'build/actuation_module/zephyr/zephyr.elf')
      )
    except AvhAPIException as e:
      print('Exception when calling v1_create_image: %s\n' % e)
      exit(1)
    print('Uploaded image ' + api_response.id + '.')

    # Reset the instance to use the new software
    print('Resetting VM to use the new software')
    try:
        api_response = await api_instance.v1_reboot_instance(instanceId)
        print('Reboot initiated successfully.')
        if hasattr(api_response, 'state') and api_response.state:
            print('Instance ' + instanceId + ' is ' + api_response.state + '.')
        else:
            print('Reboot response received, checking current state...')
    except AvhAPIException as e:
        print('Exception when calling v1_reboot_instance: %s\n' % e)
        exit(1)
    except Exception as e:
        print(f'Unexpected error during reboot: {e}')
        exit(1)

    # Wait for the instance to be ready
    count = 0
    secondsToPause = 3
    print('Waiting for the instance to boot up ...')
    
    # Get the current state after reboot
    try:
        api_response = await api_instance.v1_get_instance_state(instanceId)
        state = api_response
        print('Current state after reboot: ' + state)
    except Exception as e:
        print(f'Error getting initial state: {e}')
        exit(1)
    
    # Wait while the instance is not in a ready state
    while state in ['creating', 'booting', 'rebooting', 'starting']:
        count += 1
        time.sleep(secondsToPause)
        try:
            api_response = await api_instance.v1_get_instance_state(instanceId)
            state = api_response
            print('After ' + str(secondsToPause*count) + ' seconds, the instance is ' + state + '.')
        except Exception as e:
            print(f'Error checking instance state: {e}')
            break
    
    print('Instance is now in state: ' + state)
  
    # Connect to the console WebSocket
    print('Requesting the instance console WebSocket URL ...')
    try:
      api_response = await api_instance.v1_get_instance_console(instanceId)
      consoleWebSocketURL = api_response.url
    except AvhAPIException as e:
      print('Exception when calling v1_get_instance_console: %s\n' % e)
      exit(1)
    print('Connecting to the WebSocket ...')
    async with websockets.connect(consoleWebSocketURL) as ws:
      await handler(ws)
      await asyncio.Future()  # keep WebSocket connection open
  
async def handler(websocket):
  while True:
    message = await websocket.recv() # encoded in Unicode
    print(message.decode(encoding='UTF-8',errors='replace')) # decoded to UTF-8
  
if __name__ == '__main__':
  print('Starting the script ...')
  try:
    asyncio.run(asyncio.wait_for(main(), 7200)) # The timeout is 2 hours.
  except Exception as e:
    print('Finished running main() due to AsyncIO timeout.')
  print('Finished the script.')
  exit(0)