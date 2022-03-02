using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;
using UnityEngine.UI;
using System.Threading.Tasks;

public class KinectDepthSender : MonoBehaviour
{
    public SignalClient client;
    public NetworkManager networkManager;
    public RenderTexture output;
    public float SendRate = 0.5f;
    public int sensorIndex = 0;
    public InputField INP_DepthSendTime;

    Texture buffTex;

    Queue<Texture2D> queueTex;
    int maxTexQueueCount = 10;

    bool toSaveImage = false;
    int saveIndex = 0;

    byte[] buffByte;
    int sendCount = 0;

    IEnumerator Start()
    {
        queueTex = new Queue<Texture2D>();
        INP_DepthSendTime.onValueChanged.AddListener(x => {
            float result = 0;
            if(float.TryParse(INP_DepthSendTime.text, out result)){
                SendRate = result;
                SystemConfig.Instance.SaveData("DepthRate", SendRate);
            }
        });
        INP_DepthSendTime.text = SystemConfig.Instance.GetData<float>("DepthRate", 0.5f).ToString();

        yield return new WaitForSeconds(3);
        
        LoopUploadImage();
        LoopSend();

        while (true)
        {
            ParseDataToSend();
            yield return new WaitForSeconds(Mathf.Max(0.2f, SendRate));
        }
    }

    async void LoopUploadImage(){
        while(this != null){
            PostDepth();

            int sendTime = Mathf.Max(200, Mathf.FloorToInt(SendRate * 1000 * 1.5f));
            await Task.Delay(sendTime);
        }
    }

    void PostDepth(){
        //string url = networkManager.serverURL + networkManager.api_upload_depth;
        //Debug.Log($"post to {url}");

        if(buffByte != null)
            networkManager.API_Depth_UploadFile(buffByte, "depthImage.jpg");
    }

    async void LoopSend(){
        while(this != null){
            await Task.Run(delegate {
                try
                {
                    client.SocketSend(buffByte);
                    sendCount++;
                    //Debug.Log($"total byte {buffByte.Length} send. (" + (float)buffByte.Length / 1024f + "KB)");
                }
                catch (System.Exception e)
                {
                    Debug.LogError("Depth Socket error: " + e);
                }
            });

            int sendTime = Mathf.FloorToInt(SendRate * 1000);
            await Task.Delay(sendTime);
        }
    }

    void ParseDataToSend()
    {
        if(buffTex == null)
            return;

        var tex = TextureToTexture2D(buffTex);
        buffByte = tex.EncodeToJPG(100);

        queueTex.Enqueue(tex);
        if(queueTex.Count > maxTexQueueCount){
            var deTex = queueTex.Dequeue();
            Destroy(deTex);
        }

        try {
            if(toSaveImage){
                WriteFile(buffByte);
                //PostDepth();
                toSaveImage = false;
            }
        } catch {}
    }

    void Update()
    {
        KinectManager kinectManager = KinectManager.Instance;
        if (kinectManager && kinectManager.IsInitialized())
        {
            buffTex = kinectManager.GetDepthImageTex(sensorIndex);
            Graphics.Blit(buffTex, output);
        }

        if(buffByte != null)
            DeviceLogger.instance.SetText_ToSendImageByte($"total byte {buffByte.Length} send. (" + (float)buffByte.Length / 1024f + "KB)" + $" (id:{sendCount})");

        if(Input.GetKeyDown(KeyCode.F12)){
            toSaveImage = true;
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

    void WriteFile(byte[] data){
        var path = Application.dataPath + $"/../output_{saveIndex}.jpg";
        System.IO.File.WriteAllBytes(path, data);
        //networkManager.API_Depth_UploadFile(path, "depthImage.jpg");
        saveIndex++;
    }
}
