using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandInfos : MonoBehaviour
{
    public ArduinoInteractive ardiono;
    void Start()
    {
        ardiono.OnRecieveData += OnRecieveHand;
    }

    void OnRecieveHand(string hand){
        string[] substrings = hand.Split (',');

        string angle = substrings[0];
        string height = substrings[1];
    }
}
