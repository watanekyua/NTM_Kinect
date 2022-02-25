using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.IO;

public class RenderSender : MonoBehaviour
{
    public SignalClient client;
    public RenderTexture source;
    public float SendRate = 0.2f;

    Texture2D buffTex = null;

    IEnumerator Start()
    {
        yield return new WaitForSeconds(3);

        while (true)
        {
            ParseDataToSend();
            yield return new WaitForSeconds(SendRate);
        }
        
    }

    void ParseDataToSend(){
        if(buffTex == null)
            buffTex = new Texture2D(source.width, source.height, TextureFormat.RGB24, false);

        source.enableRandomWrite = true;
        RenderTexture.active = source;
        buffTex.ReadPixels(new Rect(0, 0, source.width, source.height), 0, 0);
        RenderTexture.active = null;

        byte[] bytes = buffTex.EncodeToJPG(100);

        client.SocketSend(bytes);

        Debug.Log($"total byte {bytes.Length} send. (" + (float)bytes.Length / 1024f + "KB)");
        DeviceLogger.instance.SetText_ToSendImageByte(($"total byte {bytes.Length} send. (" + (float)bytes.Length / 1024f + "KB)"));
        //WriteFile(bytes);
    }

    void Update(){
        if(Input.GetKeyDown(KeyCode.Space)){
            ParseDataToSend();
        }
    }

    // void WriteFile(byte[] data){
    //     var path = Application.dataPath + "/../output.txt";
    //     File.WriteAllBytes(path, data);
    // }
}
