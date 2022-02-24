using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PosReciever : MonoBehaviour
{
    public SignalClient client;
    public bool isReadyToSend = false;
    void Start()
    {
        client.OnSignalReceived.AddListener(RecieveData);
    }

    void RecieveData(string data){
        string[] tok = data.Split(",");
        try {
            float x, y;
            float.TryParse(tok[0], out x);
            float.TryParse(tok[1], out y);

            if(isReadyToSend)
                LogicManager.instance.SendSerial(new Vector2(x, y));
                
        } catch {}
    }
}
