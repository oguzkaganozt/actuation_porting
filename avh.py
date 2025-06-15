#!/usr/bin/env python3

# Copyright (c) 2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0
#
# ARM Virtual Hardware (AVH) Firmware Management Script
# Builds and manages firmware on an AVH instance.
#
# Usage: ./avh.py [OPTIONS]

import argparse
import asyncio
import os
import sys
import subprocess
from pathlib import Path
from dotenv import load_dotenv
import websockets
import avh_api_async as AvhAPI
from avh_api_async.exceptions import ApiException as AvhAPIException
from datetime import datetime

# Configuration
FIRMWARE_PATH = 'build/actuation_module/zephyr/zephyr.elf'
BOOT_TIMEOUT_SECONDS = 7200  # 2 hours
STATE_CHECK_INTERVAL = 2  # seconds

def load_config():
    """Load and validate environment variables"""
    load_dotenv()
    
    api_endpoint = os.getenv('AVH_API_ENDPOINT')
    api_token = os.getenv('AVH_API_TOKEN')
    project_name = os.getenv('AVH_PROJECT_NAME')
    instance_name = os.getenv('AVH_INSTANCE_NAME')
    instance_flavor = os.getenv('AVH_INSTANCE_FLAVOR')
    
    if not all([api_token, api_endpoint, project_name, instance_name, instance_flavor]):
        print("❌ Missing required environment variables:")
        print("   - AVH_API_TOKEN")
        print("   - AVH_API_ENDPOINT") 
        print("   - AVH_PROJECT_NAME")
        print("   - AVH_INSTANCE_NAME")
        print("   - AVH_INSTANCE_FLAVOR")
        sys.exit(1)
          
    return api_endpoint, api_token, project_name, instance_name, instance_flavor


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


async def get_project_id(api_instance, project_name):
    """Get project ID"""
    print("   Finding the project ...")
    api_response = await api_instance.v1_get_projects(name=project_name)
    projectId = api_response[0].id
    print(f"✅ Project found: {projectId}")
    return projectId


async def connect_vpn(api_instance, project_id):
    """Connect to VPN"""
    print(f"   Connecting to VPN for project: {project_id}")
    
    try:
        vpn_config = await api_instance.v1_get_project_vpn_config(project_id, format='ovpn')
        with open('avh.ovpn', 'w') as f:
            f.write(vpn_config)
            
        print(f"   VPN config written to avh.ovpn")
        subprocess.Popen(['openvpn', '--log', 'vpn.log', '--config', './avh.ovpn'])

        # Wait for VPN to connect (check if we have tap0 interface)
        while True:
            result = subprocess.run(['ip', 'addr', 'show', 'tap0'], capture_output=True, text=True)
            if 'tap0' in result.stdout:
                print("   VPN connected")
                break
            await asyncio.sleep(1)

    except AvhAPIException as e:
        print(f"❌ VPN connection failed: {e}")
        sys.exit(1)


async def disconnect_vpn():
    """Disconnect from VPN"""
    print("   Disconnecting from VPN")
    subprocess.run(['pkill', 'openvpn'])


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
        await reboot_instance(api_instance, instance_id)
        await wait_for_ready(api_instance, instance_id)
        
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
        logs_dir = Path("log")
        logs_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = logs_dir / f"{timestamp}.log.ansi"
        
        with open(log_filename, "w", encoding='utf-8') as log_file:
            os.chmod(log_filename, 0o666)
            while True:
                message = await websocket.recv()
                decoded_message = message.decode('utf-8', errors='replace')
                print(decoded_message, end='')
                log_file.write(decoded_message)
                log_file.flush()  # Ensure immediate write to file
    except websockets.exceptions.ConnectionClosed:
        print("\nConsole connection closed")
    except KeyboardInterrupt:
        print("\nKeyboard interrupt")


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


async def build_firmware(build_type='main', clean=False):
    """Build firmware"""
    
    log_dir = Path("log")
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / "build.log.ansi"

    if clean:
        print("   Cleaning build directory")
        subprocess.run(['./build.sh', '-c'], check=True)
    
    with open(log_file, 'w') as f:
        os.chmod(log_file, 0o666)
        if build_type == 'main':
            print(f"🔨 Building main firmware (build logs: log/build.log.ansi)")
            subprocess.run(['./build.sh'], check=True, stdout=f, stderr=subprocess.STDOUT)
        elif build_type == 'test':
            print(f"🔨 Building test firmware (build logs: log/build.log.ansi)")
            subprocess.run(['./build.sh', '--unit-test'], check=True, stdout=f, stderr=subprocess.STDOUT)
        elif build_type == 'test-subscriber':
            print(f"🔨 Building DDS test subscriber firmware (build logs: log/build.log.ansi)")
            subprocess.run(['./build.sh', '--dds-subscriber'], check=True, stdout=f, stderr=subprocess.STDOUT)
        elif build_type == 'test-publisher':
            print(f"🔨 Building DDS test publisher firmware (build logs: log/build.log.ansi)")
            subprocess.run(['./build.sh', '--dds-publisher'], check=True, stdout=f, stderr=subprocess.STDOUT)
        else:
            print(f"Invalid build type: {build_type}")
            sys.exit(1)
           
    print(f"✅ Firmware built")


def parse_arguments():
    """Parse and validate command line arguments"""
    parser = argparse.ArgumentParser(
        description='AVH Firmware Management Script',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Build options
    build_group = parser.add_mutually_exclusive_group()
    build_group.add_argument('--build', 
                           nargs='?',
                           const='main',
                           choices=['main', 'test', 'test-subscriber', 'test-publisher'],
                           help='Build firmware (main, test, test-subscriber, test-publisher). Defaults to main if no choice specified.')
    build_group.add_argument('--rebuild', 
                           nargs='?',
                           const='main',
                           choices=['main', 'test', 'test-subscriber', 'test-publisher'],
                           help='Rebuild firmware from clean state. Defaults to main if no choice specified.')
    
    # AVH operations
    parser.add_argument('--test-auth', action='store_true', help='Test AVH API authentication')
    parser.add_argument('--vpn-connect', action='store_true', help='Connect to VPN')
    parser.add_argument('--vpn-disconnect', action='store_true', help='Disconnect from VPN')
    parser.add_argument('--deploy', action='store_true', help='Deploy firmware to instance')
    parser.add_argument('--reboot', action='store_true', help='Reboot instance')
    parser.add_argument('--ssh', action='store_true', help='Connect to instance console')
    
    args = parser.parse_args()
    
    actions = [args.build, args.rebuild, args.vpn_connect, args.vpn_disconnect, args.deploy, args.reboot, args.ssh, args.test_auth]
    if not any(actions):
        parser.error("No action specified. Use --help to see available options.")
    
    return args


async def main(args):
    """Main script execution"""
    print("=" * 40)
    print("AVH Firmware Management Script")
    print("=" * 40)

    # Disconnect VPN
    if args.vpn_disconnect:
        await disconnect_vpn()
        return 0

    # Build firmware
    if args.build:
        await build_firmware(args.build)
    elif args.rebuild:
        await build_firmware(args.rebuild, clean=True)

    # Check if any AVH operations are requested
    avh_operations = [args.vpn_connect, args.deploy, args.reboot, args.ssh, args.test_auth]
    if not any(avh_operations):
        return 0
    
    # Setup API client
    api_endpoint, api_token, project_name, instance_name, instance_flavor = load_config()
    configuration = AvhAPI.Configuration(host=api_endpoint)
    
    async with AvhAPI.ApiClient(configuration=configuration) as api_client:        
        # Authenticate
        api_instance = AvhAPI.ArmApi(api_client)
        access_token = await authenticate(api_instance, api_token)

        # Test authentication
        if args.test_auth:
            print("✅ Authentication test successful")
            return 0

        configuration.access_token = access_token
        project_id = await get_project_id(api_instance, project_name)

        # Find instance or create new instance if not found
        instance_id = await find_instance(api_instance, instance_name, instance_flavor)
        if instance_id is None:
            instance_id = await create_instance(api_instance, project_id, instance_name, instance_flavor)

        # Connect VPN
        if args.vpn_connect:
            await connect_vpn(api_instance, project_id)
            return 0
        
        # Deploy new firmware
        if args.deploy:
            await upload_firmware(api_instance, instance_id)
        
        # Reboot instance
        if args.reboot:
            await reboot_instance(api_instance, instance_id)
            await wait_for_ready(api_instance, instance_id)

        # Connect console
        if args.ssh:
            await connect_to_console(api_instance, instance_id)


if __name__ == '__main__':
    try:
        args = parse_arguments()
        asyncio.run(asyncio.wait_for(main(args), timeout=BOOT_TIMEOUT_SECONDS))
    except asyncio.TimeoutError:
        print(f"\nScript timed out after {BOOT_TIMEOUT_SECONDS} seconds")
    except KeyboardInterrupt:
        print("\nScript interrupted")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        print("\nDone")
