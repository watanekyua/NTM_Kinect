using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;
using System.IO;
using System.Threading;
using System.Net.Sockets;
using UnityEngine.Events;
using System.Threading.Tasks;
using System.Linq;

public class SignalClient : MonoBehaviour {
    public TMPro.TextMeshProUGUI TXT_Logger;
    public string serverIP = "127.0.0.1";
    public int serverPort = 25568;
    public int recvBufferSize = 1024;
    public string EndToken = "[/TCP]";

    [HimeLib.HelpBox] public string tip = "所有的訊息接編碼為UTF-8";
    public SocketSignalEvent OnSignalReceived;


    [Header("Auto Work")]
    public bool runInStart = false;



	TcpClient tcpSocket;
	NetworkStream netStream;
    string [] token;
	Thread connectThread;
    Action ActionQueue;

	async void Start()
    {
		token = new string[]{ EndToken };

        await Task.Delay(1000);

        if(this == null) return;

        if(!runInStart) return;

        InitSocket();
	}

    void Update(){
        if(ActionQueue != null){
            try {
                ActionQueue?.Invoke();
                }
            catch( System.Exception e) {

            }
            ActionQueue = null;
        }
    }

	public async void InitSocket()
	{
        bool connect = false;

        await Task.Run(delegate {
            try
            {
                tcpSocket = new TcpClient(serverIP, serverPort);
                tcpSocket.ReceiveBufferSize = recvBufferSize;

                netStream = tcpSocket.GetStream();

                connect = true;
            }
            catch (Exception e)
            {
                Debug.LogError("Socket error: " + e);
                connect = false;
            }
        });
        
        if(!connect){
            RestartSocket();
            return;
        }

        //開啟一個線程連接，必須的，否則主線程卡死  
        connectThread = new Thread (ClientWork);
        connectThread.Start ();

        Debug.Log ($"<color=cyan>Connect to Server ({serverIP}) at port :{serverPort}</color>");
        if(TXT_Logger)
            TXT_Logger.text = $"Connect to Server ({serverIP}) at port :{serverPort}";
	}


	//Glass Client just need to Receive Touch data from Server
	void ClientWork()
	{
		while (true) {
            
            byte[] data = new byte[tcpSocket.ReceiveBufferSize];
            int bytes = 0;
            
            try
            {
                // *** networkStream.Read will let programe get Stuck ***
                bytes = netStream.Read(data, 0, (int)tcpSocket.ReceiveBufferSize);
            }
            catch (System.IO.IOException)
            {
                Debug.LogError("Disconnect Exception. Try to Reconned...");
                ActionQueue += delegate {
                    RestartSocket();
                };
                break;
            }

            if(bytes == 0)
            {
                Debug.Log("Disconnect Action. Try to Reconned...");
                ActionQueue += delegate {
                    RestartSocket();
                };
                break;
            }

            string responseData = System.Text.Encoding.UTF8.GetString(data, 0, bytes);

            if(string.IsNullOrEmpty(token[0])){
                ActionQueue += delegate {
                    OnSignalReceived.Invoke(responseData);
                };
            } else {
                string[] substrings = responseData.Split (token, StringSplitOptions.None);  // => 245,135,90

                // rs,x,y
                if (substrings.Length > 1) {
                    //Debug.Log($"TCP >> Recieved : {substrings[0]}");

                    ActionQueue += delegate {
                        OnSignalReceived.Invoke(substrings[0]);
                    };
                }
            }
		}
	}

	public void SocketSend(string sendStr)
	{
        if(tcpSocket == null)
            return;

        if(!netStream.CanWrite)
            return;

        try {
            string toSend = sendStr + EndToken;
            byte[] sendData = System.Text.Encoding.UTF8.GetBytes(toSend);
            netStream.Write(sendData, 0, sendData.Length);

            Debug.Log ($"TCP >> Send: {sendStr}");
        }
        catch(System.Exception e){
            Debug.LogError(e.Message.ToString());
        }
	}

    public void SocketSend(byte[] sendbyte)
    {
        if(tcpSocket == null)
            return;
            
        if(!netStream.CanWrite)
            return;

        try {
            netStream.Write(sendbyte, 0, sendbyte.Length);
            if(!string.IsNullOrEmpty(EndToken)){
                byte[] end = System.Text.Encoding.UTF8.GetBytes(EndToken);
                netStream.Write(end, 0, end.Length);
            }
        } catch(System.Exception e){
            Debug.LogError(e.Message.ToString());
        }
    }

	public void CloseSocket()
	{
        if(connectThread != null)
        {
            connectThread.Interrupt();
			connectThread.Abort ();
        }

        netStream?.Close();
		tcpSocket?.Close();

        Debug.Log("<color=red>Client Diconnect.</color>");
	}

    public async void RestartSocket(){
        CloseSocket();
        await Task.Delay(5000);
        if(this == null) return;
        InitSocket();
    }

    void OnApplicationQuit()
	{
		CloseSocket();
	}

    [Header("Signal Emulor")]
    public string signalForRecieved = "";
    public string signalForSend = "";
    [EasyButtons.Button] void EmuSignalRecieve(){
        OnSignalReceived?.Invoke(signalForRecieved);
    }
    [EasyButtons.Button] void EmuSignalSend(){
        SocketSend(signalForSend);
    }

    [EasyButtons.Button] void ToConnect(){
        InitSocket();
    }

    [Serializable]
    public class SocketSignalEvent : UnityEvent<string>
    {}
    
}