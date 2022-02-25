using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class DeviceLogger : HimeLib.SingletonMono<DeviceLogger>
{
    public TextMeshProUGUI TXT_RecieveMinPos;
    public TextMeshProUGUI TXT_ToSendToArduino;
    public TextMeshProUGUI TXT_RecieveArduinoAngle;
    public TextMeshProUGUI TXT_ToSendAngle;
    public TextMeshProUGUI TXT_ToSendImageByte;
    public TextMeshProUGUI TXT_Server1;
    public TextMeshProUGUI TXT_Server2;
    public TextMeshProUGUI TXT_Server3;


    public void SetText_RecieveMinPos(string msg){
        TXT_RecieveMinPos.text = msg;
    }

    public void SetText_ToSendToArduino(string msg){
        TXT_ToSendToArduino.text = msg;
    }

    public void SetText_RecieveArduinoAngle(string msg){
        TXT_RecieveArduinoAngle.text = msg;
    }

    public void SetText_ToSendAngle(string msg){
        TXT_ToSendAngle.text = msg;
    }

    public void SetText_ToSendImageByte(string msg){
        TXT_ToSendImageByte.text = msg;
    }

    public void SetText_Server1(string msg){
        TXT_Server1.text = msg;
    }
    public void SetText_Server2(string msg){
        TXT_Server2.text = msg;
    }
    public void SetText_Server3(string msg){
        TXT_Server3.text = msg;
    }
}
