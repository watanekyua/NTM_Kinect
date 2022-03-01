using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LogicManager : HimeLib.SingletonMono<LogicManager>
{
    public ArduinoInteractive arduino;
    public InputField INP_angleDelta;
    public InputField INP_Com;
    public Button BTN_Open;
    public Button BTN_Close;
    public float angleDelta = 60f;

    [Header("Arduino")]
    public bool ReadyToSend = false;
    void Start()
    {
        StartCoroutine(BeLateStart());

        BTN_Open.onClick.AddListener(() => {
            arduino.StartSerial();
        });

        BTN_Close.onClick.AddListener(() => {
            arduino.CloseArduino();
        });

        INP_angleDelta.onValueChanged.AddListener(x => {
            float result = 0;
            if(float.TryParse(INP_angleDelta.text, out result)){
                angleDelta = result;
                SystemConfig.Instance.SaveData("angleDelta", angleDelta);
            }
        });

        INP_angleDelta.text = SystemConfig.Instance.GetData<float>("angleDelta", 0).ToString();

        INP_Com.onValueChanged.AddListener(x => {
            arduino.comName = INP_Com.text;
            SystemConfig.Instance.SaveData("COM", arduino.comName);
        });

        INP_Com.text = SystemConfig.Instance.GetData<string>("COM", "COM3");
    }

    IEnumerator BeLateStart(){
        yield return new WaitForSeconds(5);
        arduino.StartSerial();
    }

    // void ParseData(string data){
    //     string[] result = data.Split("{\"data\"");
    //     string f_data;
    //     if(result.Length > 1){
    //         f_data = "{\"data\""  + result[1];

    //         //Debug.Log(f_data);
    //         var obj = JsonUtility.FromJson<PosDataBase>(f_data);

    //         float distance = 9999;
    //         PosData t = new PosData();
    //         foreach (var item in obj.data)
    //         {
    //             float dis = Mathf.Sqrt((item.x * item.x) + (item.y + item.y));
    //             if(dis < distance){
    //                 distance = dis;
    //                 t = item;
    //             }
    //         }

    //         if(distance < 9999){
    //             Debug.Log($">> {t.x}, {t.y}");
                
    //             currentPos = t;
    //             // if(ReadyToSend)
    //             //     SendSerial(new Vector2(t.x, t.y));
    //         }
            
    //         //Debug.Log(obj.data.Count);
    //     }
        
    // }

    public Vector2 testPos;
    [EasyButtons.Button]
    void SendPosToArduino(){
        SendSerial(testPos);
    }

    public void SendSerial(Vector2 pos){
        //pos = testPos;
        if(!ReadyToSend){
            Debug.Log($"Emu arduino send : {pos}");
            return;
        }
        float angle = Mathf.Atan2(pos.y, pos.x) * Mathf.Rad2Deg;
        angle = angle + angleDelta;
        angle = angle < 0 ? angle + 360f : angle;
        
        byte convAngle = (byte)Mathf.FloorToInt((angle/360)*200);
        
        byte[] tosend = {0x00, 0x06, 0x01, convAngle, 0x0d, 0x0a};
        arduino.SendByte(tosend);

        Debug.Log($"{0x00} {0x06} {0x01} {convAngle} {0x0d} {0x0a} / angle:{angle}");
        DeviceLogger.instance.SetText_ToSendToArduino($"{0x00} {0x06} {0x01} {convAngle} {0x0d} {0x0a} / angle:{angle}");
    }
}
