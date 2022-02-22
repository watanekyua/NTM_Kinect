using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class DvTCPServer : MonoBehaviour
{

    //負責接收外部訊號
    public static DvTCPServer instance;
    public static event Action OnRecieveTouchStart;

    public int serverPort = 25568;

    Socket serverSocket; //服務器端socket  
    Socket clientSocket; //客戶端socket  
    IPEndPoint ipEnd; //偵聽端口  
    string recvStr; //接收的字符串
    string sendStr; //發送的字符串
    byte[] recvData = new byte[1024]; //接收的數據，必須為字節  
    byte[] sendData = new byte[1024]; //發送的數據，必須為字節  
    int recvLen; //接收的數據長度  
    Thread connectThread; //連接線程  

    void Awake()
    {
        instance = this;
    }

    // Use this for initialization
    void Start()
    {
        Debug.Log("Start Position Server at :" + serverPort);
        InitSocket();
    }

    void OnApplicationQuit()
    {
        SocketQuit();
    }

    void InitSocket()
    {
        //定義偵聽端口,偵聽任何IP  
        ipEnd = new IPEndPoint(IPAddress.Any, serverPort);
        //定義套接字類型,在主線程中定義
        serverSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        serverSocket.Bind(ipEnd);
        //開始偵聽,最大10個連接  
        serverSocket.Listen(1);

        //開啟一個線程連接，必須的，否則主線程卡死  
        connectThread = new Thread(ServerWork);
        connectThread.Start();
    }

    void ServerWork()
    {
        //連接
        SocketConnet();
        //進入接收循環  
        while (true)
        {
            //對data清零  
            recvData = new byte[1024];
            try
            {
                //獲取收到的數據的長度  
                recvLen = clientSocket.Receive(recvData);
            }
            catch (System.Net.Sockets.SocketException)
            {
                SocketConnet();
                continue;
            }
            //如果收到的數據長度為0，則重連並進入下一個循環  
            if (recvLen == 0)
            {
                SocketConnet();
                continue;
            }
            //輸出接收到的數據  
            recvStr = Encoding.UTF8.GetString(recvData, 0, recvLen);

            //N[/TCP]
            Debug.Log(recvStr);

            //Recieve Data Will Be   245,135,90[/TCP]   , str 不會包含[/TCP]
            char delimiterEnd = '[';
            string[] clearString = recvStr.Split(delimiterEnd);  // => N , [/TCP]

            if (clearString.Length > 1)
            {
                Debug.Log("N:" + clearString[0]);

                int DataNum = 0;
                int.TryParse(clearString[0], out DataNum);

                if (DataNum == 0)
                    continue;

                OnRecieveTouchStart?.Invoke();

            } // end Length

        }  // end While
    }

    void SocketConnet()
    {
        if (clientSocket != null)
            clientSocket.Close();
        //控制台輸出偵聽狀態
        print("Waiting for a client");
        //一旦接受連接，創建一個客戶端  
        clientSocket = serverSocket.Accept();
        //獲取客戶端的IP和端口  
        IPEndPoint ipEndClient = (IPEndPoint)clientSocket.RemoteEndPoint;
        //輸出客戶端的IP和端口  
        Debug.Log("Connect with " + ipEndClient.Address.ToString() + ":" + ipEndClient.Port.ToString());

        //連接成功則發送數據  
        //sendStr="Welcome to my server";
        //SocketSend(sendStr);  
    }

    //Data to Glass can use UTF8
    public void SocketSend(string sendStr)
    {
        if (clientSocket == null)
            return;
        if (clientSocket.Connected == false)
            return;
        try {
            sendStr = sendStr + "[/TCP]";
            //清空發送緩存  
            sendData = new byte[1024];
            //數據類型轉換  
            sendData = Encoding.UTF8.GetBytes(sendStr);
            //發送  
            clientSocket.Send(sendData, sendData.Length, SocketFlags.None);
        }
        catch(System.Exception e){
            Debug.Log(e.Message.ToString());
        }
    }

    void SocketQuit()
    {
        //先關閉客戶端  
        if (clientSocket != null)
            clientSocket.Close();
        //再關閉線程  
        if (connectThread != null)
        {
            connectThread.Interrupt();
            connectThread.Abort();
        }
        //最後關閉服務器
        if (serverSocket != null)
        {
            serverSocket.Close();
            print("diconnect");
        }
    }

    public static void FakeInvoke(){
        OnRecieveTouchStart?.Invoke();
    }
}