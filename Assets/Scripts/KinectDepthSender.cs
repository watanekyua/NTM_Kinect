using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;

public class KinectDepthSender : MonoBehaviour
{
    public SignalClient client;
    public RenderTexture output;
    public float SendRate = 0.5f;
    public int sensorIndex = 0;

    Texture buffTex;

    IEnumerator Start()
    {
        yield return new WaitForSeconds(3);

        while (true)
        {
            ParseDataToSend();
            yield return new WaitForSeconds(SendRate);
        }

    }

    void ParseDataToSend()
    {
        if(buffTex == null)
            return;

        byte[] bytes = TextureToTexture2D(buffTex).EncodeToJPG(100);

        client.SocketSend(bytes);

        Debug.Log($"total byte {bytes.Length} send. (" + (float)bytes.Length / 1024f + "KB)");
        DeviceLogger.instance.SetText_ToSendImageByte(($"total byte {bytes.Length} send. (" + (float)bytes.Length / 1024f + "KB)"));
    }

    void Update()
    {
        KinectManager kinectManager = KinectManager.Instance;
        if (kinectManager && kinectManager.IsInitialized())
        {
            buffTex = kinectManager.GetDepthImageTex(sensorIndex);
            Graphics.Blit(buffTex, output);
        }
    }

    private Texture2D TextureToTexture2D(Texture texture)
    {
        Texture2D texture2D = new Texture2D(texture.width, texture.height, TextureFormat.RGBA32, false);
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture renderTexture = RenderTexture.GetTemporary(texture.width, texture.height, 32);
        Graphics.Blit(texture, renderTexture);

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture2D.Apply();

        RenderTexture.active = currentRT;
        RenderTexture.ReleaseTemporary(renderTexture);

        return texture2D;
    }
}
