import sys
import time
from .NatNetClient import NatNetClient
import numpy as np

class OptiTrackDataStreamer():
    def __init__(self, buf_size = 100):
        self.poses = []
        self.rots = []
        self.times = []
        self.buf_size = 1000

        optionsDict = {}
        optionsDict["clientAddress"] = "127.0.0.1"
        optionsDict["serverAddress"] = "127.0.0.1"
        optionsDict["use_multicast"] = True

        streaming_client = NatNetClient()
        streaming_client.set_client_address(optionsDict["clientAddress"])
        streaming_client.set_server_address(optionsDict["serverAddress"])
        streaming_client.set_use_multicast(optionsDict["use_multicast"])

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.new_frame_listener = self.receive_new_frame
        streaming_client.rigid_body_listener = self.receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()
        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")

        time.sleep(1)
        if streaming_client.connected() is False:
            print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
            try:
                sys.exit(2)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        
        self.streaming_client = streaming_client
    
    def buf_add(self,l,x):
        #append x to l, limiting len(l) to be buf_size 
        l.append(x)
        if len(l) == self.buf_size:
            l = l[1:]
    
    def get_closest_datapoint(self,time):
        #convert to cm

        if len(self.times) == 0:
            return None,None,None
        ind = np.argmin(abs(np.subtract(self.times,time)))
        pos = self.poses[ind]
        rot = self.rots[ind]
        time = self.times[ind]
        return np.array(pos)*100,np.array(rot),time
        

    # This is a callback function that gets connected to the NatNet client. 
    # It is called once per rigid body per frame
    def receive_rigid_body_frame(self, new_id, position, rotation ):
        #print( "Received frame for rigid body", new_id )
        #print( "Received frame for rigid body", new_id," ",position," ",rotation )
        self.buf_add(self.poses,position)
        self.buf_add(self.rots,rotation)
        self.buf_add(self.times,time.time())
    
    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(self,data_dict):
        return
        # order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
        #             "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        # dump_args = False
        # if dump_args == True:
        #     out_string = "    "
        #     for key in data_dict:
        #         out_string += key + "="
        #         if key in data_dict :
        #             out_string += data_dict[key] + " "
        #         out_string+="/"
        #     print(out_string)

    def close(self):
        self.streaming_client.shutdown()