using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UploadRunner : MonoBehaviour
{
    public NetworkManager networkManager;
    public string filePath;
    void Start()
    {
        filePath = $"{Application.dataPath}/../output.jpg";
        Debug.Log(filePath);
        Debug.Log(System.IO.File.Exists(filePath));
        networkManager.API_Depth_UploadFile(filePath, "depth.jpg");
    }
}
