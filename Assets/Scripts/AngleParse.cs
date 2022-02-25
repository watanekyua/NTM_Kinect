using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AngleParse : MonoBehaviour
{
    public ArduinoInteractive arduino;
    public SignalClient arduinoSender;
    void Start()
    {
        arduino.OnRecieveData += RecvData;
    }

    void RecvData(string data){
        DeviceLogger.instance.SetText_RecieveArduinoAngle(data);
        arduinoSender.SocketSend(data);
        DeviceLogger.instance.SetText_ToSendAngle(data);
    }
}
