using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class AngleParse : MonoBehaviour
{
    public ArduinoInteractive arduino;
    public SignalClient arduinoSender;
    public NetworkManager networkManager;
    public int sendTime = 333;
    string arduinoAngle;
    string uploadAngleResult;

    int sendCount = 0;
    
    void Start()
    {
        arduino.OnRecieveData += RecvData;

        LoopUploadAngle();
        LoopSend();
    }

    void Update(){
        DeviceLogger.instance.SetText_RecieveArduinoAngle($"{arduinoAngle} (id:{sendCount})");
        DeviceLogger.instance.SetText_PostAngleResult($"{uploadAngleResult}");
    }

    void RecvData(string data){
        arduinoAngle = data;
    }

    // void RecvData2(string data){
    //     DeviceLogger.instance.SetText_RecieveArduinoAngle(data);
    //     arduinoSender.SocketSend(data);
    //     DeviceLogger.instance.SetText_ToSendAngle(data);
    // }

    async void LoopUploadAngle(){
        while(this != null){
            string url = networkManager.serverURL + networkManager.api_upload_angle + $"?angle={arduinoAngle}";
            networkManager.API_GetURL(url, x => {
                uploadAngleResult = $"{url} ({x})";
            });
            await Task.Delay(sendTime);
        }
    }

    async void LoopSend(){
        while(this != null){
            await Task.Run(delegate {
                try
                {
                    arduinoSender.SocketSend(arduinoAngle);
                    sendCount++;
                }
                catch (System.Exception e)
                {
                    Debug.LogError("Angle Socket error: " + e);
                }
            });

            await Task.Delay(sendTime);
        }
    }
}
