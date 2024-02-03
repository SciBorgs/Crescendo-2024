import json
import time
import ntcore
import trajectory

SERVER_IP = ""
DEVICE_ID = 0
def config_update(event):
    global trajectory
    print("Creating Trajectory")
    config = json.loads(event.data.value.getString())
    trajectory = trajectory(config)

def request_update(event):
    global trajectory
    
if __name__ == "__main__":
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.setServer(SERVER_IP)
    nt_inst.startClient4("Aios" + str(DEVICE_ID))
    config_sub = nt_inst.getStringTopic("/aios/config").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    request_sub = nt_inst.getStringTopic("/aios/request").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    result_pub = nt_inst.getDoubleArrayTopic(
        "" + str(DEVICE_ID)
    ).publish(ntcore.PubSubOptions(periodic=0))
    ping_pub = nt_inst.getIntegerTopic("/aios/result" + str(DEVICE_ID)).publish(
        ntcore.PubSubOptions()
    )

    # Create event listeners
    nt_inst.addListener(config_sub, ntcore.EventFlags.kValueRemote, config_update)
    nt_inst.addListener(request_sub, ntcore.EventFlags.kValueRemote, request_update)

    # Run forever
    i = 0
    while True:
        time.sleep(0.1)
        i = (i + 1) % 10
        ping_pub.set(i)
    