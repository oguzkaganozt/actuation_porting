#!/usr/bin/env python3
"""
ARM Virtual Hardware (AVH) Firmware Upload Script
Uploads firmware to an AVH instance and connects to its console.
"""

import asyncio
import os
import sys
import subprocess
from pathlib import Path
from dotenv import load_dotenv
import websockets
import avh_api_async as AvhAPI
from avh_api_async.rest import ApiException as AvhAPIException

# Configuration
FIRMWARE_PATH = 'build/actuation_module/zephyr/zephyr.elf'
BOOT_TIMEOUT_SECONDS = 7200  # 2 hours
STATE_CHECK_INTERVAL = 2  # seconds

def load_config():
    """Load and validate environment variables"""
    load_dotenv()
    
    api_endpoint = os.getenv('AVH_API_ENDPOINT')
    api_token = os.getenv('AVH_API_TOKEN')
    instance_name = os.getenv('AVH_INSTANCE_NAME')
    instance_flavor = os.getenv('AVH_INSTANCE_FLAVOR')
    
    if not all([api_token, api_endpoint, instance_name, instance_flavor]):
        print("❌ Missing required environment variables:")
        print("   - AVH_API_TOKEN")
        print("   - AVH_API_ENDPOINT") 
        print("   - AVH_INSTANCE_NAME")
        print("   - AVH_INSTANCE_FLAVOR")
        sys.exit(1)
          
    return api_endpoint, api_token, instance_name, instance_flavor


async def authenticate(api_instance, api_token):
    """Login to AVH API"""
    print("   Logging in...")
    
    try:
        token_response = await api_instance.v1_auth_login({"apiToken": api_token})
        print("✅ Logged in successfully")
        return token_response.token
    except AvhAPIException as e:
        print(f"❌ Login failed: {e}")
        sys.exit(1)


async def get_project_id(api_instance):
    """Get project ID"""
    print("   Finding the project ...")
    api_response = await api_instance.v1_get_projects()
    projectId = api_response[0].id
    print(f"✅ Project found: {projectId}")
    return projectId


async def connect_to_vpn(api_instance, project_id):
    """Connect to VPN"""
    print(f"   Connecting to VPN for project: {project_id}")
    
    try:
        vpn_config = await api_instance.v1_get_project_vpn_config(project_id, format='ovpn')
        
        # Write VPN config to file
        with open('avh.ovpn', 'w') as f:
            f.write(vpn_config)
            
        # Connect to VPN
        subprocess.run(['openvpn', './avh.ovpn'], check=True)
        print(f"✅ VPN connected: {vpn_config.url}")

    except AvhAPIException as e:
        print(f"❌ VPN connection failed: {e}")
        sys.exit(1)


async def create_instance(api_instance, project_id, instance_name, instance_flavor):
    """Create instance"""
    print(f"   Creating new instance: {instance_name} with flavor: {instance_flavor}")
    
    try:
        instance = await api_instance.v1_create_instance({
            'project': project_id,
            'name': instance_name,
            'flavor': instance_flavor,
            'os': '1.0.0'
        })
        print(f"✅ Instance created: {instance.id}")
        await asyncio.sleep(2)
        return instance.id
    except AvhAPIException as e:
        print(f"❌ Instance creation failed: {e}")
        sys.exit(1)


async def find_instance(api_instance, instance_name, instance_flavor):
    """Find instance by name and return its details"""
    print(f"   Looking for instance: {instance_name}")
    
    try:
        instances = await api_instance.v1_get_instances(name=instance_name)
        
        if not instances:
            print(f"❌ No instance found with name: {instance_name}")
            return None
        
        for instance in instances:
            if instance.flavor == instance_flavor:
                print(f"✅ Found instance: {instance.id}")
                print(f"    Service IP: {instance.service_ip}")
                print(f"    Flavor: {instance.flavor}")
                print(f"    State: {instance.state}")
                print(f"    Project: {instance.project}")
                return instance.id
            
        print(f"❌ No instance found with flavor: {instance_flavor}")
        return None
        
    except AvhAPIException as e:
        print(f"❌ Failed to find instance: {e}")
        sys.exit(1)


async def upload_firmware(api_instance, instance_id):
    """Upload firmware to the instance"""
    firmware_path = Path(FIRMWARE_PATH)
    
    if not firmware_path.exists():
        print(f"❌ Firmware file not found: {firmware_path}")
        sys.exit(1)
        
    print(f"📤 Uploading firmware: {firmware_path}")
    
    try:
        response = await api_instance.v1_create_image(
            'fwbinary', 'plain',
            name="zephyr.elf",
            instance=instance_id,
            file=str(firmware_path.absolute())
        )
        print(f"✅ Firmware uploaded: {response.id}")
        
    except AvhAPIException as e:
        print(f"❌ Upload failed: {e}")
        sys.exit(1)


async def reboot_instance(api_instance, instance_id):
    """Reboot instance with new firmware"""
    print("🔄 Rebooting instance...")
    
    try:
        await api_instance.v1_reboot_instance(instance_id)
        print("   Reboot initiated")
    except AvhAPIException as e:
        print(f"❌ Reboot failed: {e}")
        sys.exit(1)


async def wait_for_ready(api_instance, instance_id):
    """Wait for instance to finish booting"""
    print("⏳ Waiting for instance to boot...")
    
    transitional_states = {'creating', 'booting', 'rebooting', 'starting'}
    elapsed_time = 0
    
    while True:
        try:
            state = await api_instance.v1_get_instance_state(instance_id)
            
            if state not in transitional_states:
                print(f"✅ Instance ready! State: {state}")
                break
                
            print(f"    State: {state} (after {elapsed_time}s)")
            await asyncio.sleep(STATE_CHECK_INTERVAL)
            elapsed_time += STATE_CHECK_INTERVAL
            
        except Exception as e:
            print(f"❌ Error checking state: {e}")
            sys.exit(1)


async def monitor_console(websocket):
    """Display console output"""
    try:
        while True:
            message = await websocket.recv()
            decoded_message = message.decode('utf-8', errors='replace')
            print(decoded_message, end='')
    except websockets.exceptions.ConnectionClosed:
        print("\nConsole connection closed")
    except KeyboardInterrupt:
        print("\nDisconnecting from console...")


async def connect_to_console(api_instance, instance_id):
    """Connect to instance console WebSocket"""
    print("   Connecting to console...")
    
    try:
        console_response = await api_instance.v1_get_instance_console(instance_id)
        websocket_url = console_response.url
        
        print("🖥️  Console connected (Ctrl+C to exit)")
        print("-" * 50)
        
        async with websockets.connect(websocket_url) as websocket:
            await monitor_console(websocket)
            
    except AvhAPIException as e:
        print(f"❌ Console connection failed: {e}")
        sys.exit(1)


async def main():
    """Main script execution"""
    print("=" * 40)
    print("Starting AVH Firmware Upload")
    print("=" * 40)
    
    # Load configuration
    api_endpoint, api_token, instance_name, instance_flavor = load_config()
    
    # Setup API client
    configuration = AvhAPI.Configuration(host=api_endpoint)
    
    async with AvhAPI.ApiClient(configuration=configuration) as api_client:
        api_instance = AvhAPI.ArmApi(api_client)
        
        # Authenticate
        access_token = await authenticate(api_instance, api_token)
        configuration.access_token = access_token

        # Get project ID
        project_id = await get_project_id(api_instance)

        # Connect to VPN if --vpn flag is provided
        if '--vpn' in sys.argv:
            await connect_to_vpn(api_instance, project_id)
            sys.exit(0)
        
        # Find instance or create new instance
        instance_id = await find_instance(api_instance, instance_name, instance_flavor)
        if instance_id is None:
            instance_id = await create_instance(api_instance, project_id, instance_name, instance_flavor)
        
        # Upload firmware
        await upload_firmware(api_instance, instance_id)
        
        # Reboot with new firmware
        await reboot_instance(api_instance, instance_id)
        
        # Wait for boot
        await wait_for_ready(api_instance, instance_id)
        
        # Connect to console
        await connect_to_console(api_instance, instance_id)


if __name__ == '__main__':
    try:
        asyncio.run(asyncio.wait_for(main(), timeout=BOOT_TIMEOUT_SECONDS))
    except asyncio.TimeoutError:
        print(f"\nScript timed out after {BOOT_TIMEOUT_SECONDS} seconds")
    except KeyboardInterrupt:
        print("\nScript interrupted")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        print("\nDone")