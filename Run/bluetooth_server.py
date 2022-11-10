import bluetooth
    
name="bt_server"
target_name="test"
uuid="00001101-0000-1000-8000-00805F9B34FB"

def runServer():
    serverSocket=bluetooth.BluetoothSocket(bluetooth.RFCOMM )
    port=0
    serverSocket.bind(("",port))
    print "Listening for connections on port: ", port   

    serverSocket.listen(1)
    port=serverSocket.getsockname()[1]

    bluetooth.advertise_service( serverSocket, "SampleServer",
                    service_id = uuid,
                    service_classes = [ uuid, bluetooth.SERIAL_PORT_CLASS ],
                    profiles = [ bluetooth.SERIAL_PORT_PROFILE ] 
                    )

    inputSocket, address=serverSocket.accept()
    print "Got connection with" , address
    while(1):
        data=inputSocket.recv(1024)
        print "received [%s] \n " % data    
    inputSocket.close()
    serverSocket.close()  

runServer()  
