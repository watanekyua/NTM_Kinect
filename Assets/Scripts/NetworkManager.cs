using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;

public class NetworkManager : HimeLib.SingletonMono<NetworkManager>
{
    public TMPro.TextMeshProUGUI TXT_Result;

    [Header(@"API URL")]
    public string serverURL = "https://media.iottalktw.com";
    public string api_upload_Video = "/api/face/upload/video";
    public string api_upload_Image = "/api/face/upload/image";
    public string api_get_Video = "/api/face/video";
    public string api_get_Image = "/api/face/image";
    public string api_upload_depth = "/uploadImage";
    public string api_upload_angle = "/uploadRobotData";


    public System.Action<float> OnProgressUpdate;
    bool toAbort = false;

    public void API_Depth_UploadFile(string filePath, string fileID){
        StartCoroutine(HttpPostFile(serverURL + api_upload_depth, filePath, fileID));
    }

    public void API_Depth_UploadFile(byte[] fileData, string fileID){
        StartCoroutine(HttpPostFile(serverURL + api_upload_depth, fileData, fileID));
    }

    public string API_GetImage(string fileID){
        return serverURL + api_get_Image + "/" + fileID;
    }

    public string API_GetMedia(string fileID){
        return serverURL + api_get_Video + "/" + fileID;
    }

    public void API_GetURL(string url, System.Action<string> callback){
        StartCoroutine(HttpGetFile(url, callback));
    }

    public IEnumerator HttpPostJSON(string url, string json, System.Action<string> callback)
    {
        // 這個方法會把json裡的文字編碼成url code , 例如 { 變成 %7B
        // var request = UnityWebRequest.Post(url, json);
        // request.SetRequestHeader("Content-Type", "application/json");

        var request = new UnityWebRequest(url, "POST");
        byte[] bodyRaw = System.Text.Encoding.UTF8.GetBytes(json);
        request.uploadHandler = (UploadHandler) new UploadHandlerRaw(bodyRaw);
        request.downloadHandler = (DownloadHandler) new DownloadHandlerBuffer();
        request.SetRequestHeader("Content-Type", "application/json");

        yield return request.SendWebRequest();

        if (request.isNetworkError || request.isHttpError){
            Debug.Log("Network error has occured: " + request.GetResponseHeader(""));
        } else {
            Debug.Log("Success: " + request.downloadHandler.text);
            
            callback?.Invoke(request.downloadHandler.text);
        }

        // byte[] results = request.downloadHandler.data;
        // Debug.Log("Data: " + System.String.Join(" ", results));
    }

    public IEnumerator HttpPostFile(string url, string filePath, string fileName){

        if(string.IsNullOrEmpty(url) || string.IsNullOrEmpty(filePath) || string.IsNullOrEmpty(fileName)){
            Debug.LogError($"Post file param Error. {url} / {filePath} / {fileName}");
            yield break;
        }
        
        WWWForm form = new WWWForm();
        form.AddBinaryData("file", File.ReadAllBytes(filePath), fileName);
        UnityWebRequest www = UnityWebRequest.Post(url, form);

        var asyncOp = www.SendWebRequest();
        while (!asyncOp.isDone)
        {
            if(toAbort){
                www.Abort();
                break;
            }
            OnProgressUpdate?.Invoke(asyncOp.progress);
            yield return null;
        }

        if (www.isNetworkError || www.isHttpError){
            Debug.Log(www.error);
        } else {
            Debug.Log("Form upload complete! >> :" + www.downloadHandler.text);
        }
    }

    public IEnumerator HttpPostFile(string url, byte[] fileData, string fileName){

        if(string.IsNullOrEmpty(url) || fileData == null || string.IsNullOrEmpty(fileName)){
            Debug.LogError($"Post file param Error. {url} / filePath null / {fileName}");
            yield break;
        }
        
        WWWForm form = new WWWForm();
        form.AddBinaryData("file", fileData, fileName);
        UnityWebRequest www = UnityWebRequest.Post(url, form);

        yield return www.SendWebRequest();

        if (www.result == UnityWebRequest.Result.ConnectionError || www.result == UnityWebRequest.Result.ProtocolError){
            //Debug.Log(www.error);
            if(TXT_Result) TXT_Result.text = www.error;
        } else {
            //Debug.Log("Form upload complete! >> :" + www.downloadHandler.text);
            if(TXT_Result) TXT_Result.text = "Form upload complete! >> :" + www.downloadHandler.text;
        }

        www.Dispose();
    }

    public IEnumerator HttpGetFile(string url, System.Action<string> callback){
        if(string.IsNullOrEmpty(url)){
            Debug.LogError($"Post file param Error. {url}");
            yield break;
        }

        var request = new UnityWebRequest(url, "GET");
        request.downloadHandler = (DownloadHandler) new DownloadHandlerBuffer();

        yield return request.SendWebRequest();

        if (request.isNetworkError || request.isHttpError){
            Debug.Log("Network error has occured: " + request.GetResponseHeader(""));
        } else {
            //Debug.Log("Success: " + url);
            
            callback?.Invoke(request.downloadHandler.text);
        }
    }

    public void AbortUploading(){
        toAbort = true;
    }
}
