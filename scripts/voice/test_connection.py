
import asyncio
import websockets

async def ping():
    # The IP address of your Windows desktop
    uri = "ws://192.168.0.108:8998" 
    print(f"Trying to connect to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Successfully connected to the Brain!")
    except Exception as e:
        print(f"❌ Connection failed: {e}")

asyncio.run(ping())
