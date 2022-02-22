using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LogicManager : MonoBehaviour
{
    public ArduinoInteractive arduino;
    public InputField INP_angleDelta;

    public float angleDelta = 60f;
    void Start()
    {
        SignalClient.instance.OnSignalReceived.AddListener(ParseData);
        INP_angleDelta.onValueChanged.AddListener(x => {
            float result = 0;
            if(float.TryParse(INP_angleDelta.text, out result)){
                angleDelta = result;
                SystemConfig.Instance.SaveData("angleDelta", angleDelta);
            }
        });

        INP_angleDelta.text = SystemConfig.Instance.GetData<float>("angleDelta", 0).ToString();
        
    }

    void ParseData(string data){
        var result = JsonUtility.FromJson<PosDataBase>(data);

        Debug.Log(result);
        Debug.Log(result.data);
        Debug.Log(result.data[0]);
        //Debug.Log(result.data[0][0]);
    }

    public Vector2 testPos;
    
    [EasyButtons.Button]
    void SendSerial(Vector2 pos){
        //pos = testPos;
        float angle = Mathf.Atan2(pos.y, pos.x) * Mathf.Rad2Deg;
        angle = angle < 0 ? angle + 360f : angle;
        angle = angle + angleDelta;
        byte convAngle = (byte)Mathf.FloorToInt((angle/360)*200);
        
        Debug.Log($"{angle},{convAngle}");
        byte[] tosend = {0x00, 0x06, 0x02, convAngle, 0x0d, 0x0a};
        arduino.SendByte(tosend);
    }

    public string testData;
    [ContextMenu("test")]
    void TestData(){
        ParseData(testData);
    }


    [System.Serializable]
    public class PosDataBase
    {
        public List<List<string>> data;
    }

    [System.Serializable]
    public class PosData
    {
        public List<float> num;
    }
}
