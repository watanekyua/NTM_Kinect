using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class PosEmu : MonoBehaviour
{
    public bool activeDebug = false;
    ArduinoInteractive arduino;
    
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetMouseButtonDown(0)){
            //Debug.Log(Input.mousePosition);
            Vector2 pos = new Vector2(Input.mousePosition.x - Screen.width/2, Input.mousePosition.y - Screen.height/2);
            Debug.Log(pos);

            if(activeDebug)
                LogicManager.instance.SendSerial(pos);
        }
    }
}
