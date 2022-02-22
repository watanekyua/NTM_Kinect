using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System;
using System.IO;
using System.Threading;
using System.Net.Sockets;


public class DvTCPClient : MonoBehaviour {
	public static DvTCPClient instance;
    public static Action OnRecieveGameFinished;

    public Button BTNGO;
	public InputField InputHost;
    public InputField InputPort;

    String hostIP;
	int hostPort = 25568;


	Boolean socket_ready = false;
	TcpClient tcp_socket;
	NetworkStream net_stream;
	StreamWriter socket_writer;
	StreamReader socket_reader;
	Thread clientSock;

	void Awake(){
		instance = this;
	}

	void Start(){
        InputHost.text = PlayerPrefs.GetString("HostIP", "");
        InputPort.text = PlayerPrefs.GetString("HostPort", "");
		CatchInputField();
	}

    public void SaveHostIP(string src){
		PlayerPrefs.SetString("HostIP", src);
	}
    public void SaveHostPort(string src){
		PlayerPrefs.SetString("HostPort", src);
	}

    void CatchInputField(){
		hostIP = InputHost.text;
        try {
		    hostPort = Convert.ToInt32(InputPort.text);
        } catch {}
    }

	public void StartTCP()
	{
        CatchInputField();
        BTNGO.interactable = false;
		SetupSocket (hostIP, hostPort);

        if(!socket_ready)
            return;

        Debug.Log ("Connect to Server at port :" + hostPort);
        clientSock = new Thread (ReadSocket);
        clientSock.Start ();
	}


	//Glass Client just need to Receive Touch data from Server
	void ReadSocket()
	{
		while (true) {
			Thread.Sleep (20);

			if (socket_ready) {

				Byte[] data = new Byte[256];
				String responseData = String.Empty;
				Int32 bytes = 0;
				
				try
				{
					// *** networkStream.Read will let programe get Stuck ***
					bytes = net_stream.Read(data, 0, data.Length);
				}
                catch (System.IO.IOException)
				{
					print("Disconnect Exception");
					CloseSocket();
					break;
				}

				if(bytes == 0)
				{
					print("Disconnect Action");
					CloseSocket();
					break;
				}

				responseData = System.Text.Encoding.UTF8.GetString(data, 0, bytes);

				//Recieve Data Will Be   245,135,90[/TCP]   , str 不會包含[/TCP]
				char delimiterEnd = '[';
				string[] substrings = responseData.Split (delimiterEnd);  // => 245,135,90

				// rs,x,y
				if (substrings.Length > 1) {
                    // int DataNum = 0;
					// int.TryParse (substrings [0], out DataNum);

					// if (DataNum == 0)
					// 	continue;
                    //substrings [0] == "pt"

					OnRecieveGameFinished?.Invoke();
				}
			}
		}
	}

	public void SetupSocket(string host, int port)
	{
		try
		{
			tcp_socket = new TcpClient(host, port);

			net_stream = tcp_socket.GetStream();
			socket_writer = new StreamWriter(net_stream);
			socket_reader = new StreamReader(net_stream);

			socket_ready = true;
		}
		catch (Exception e)
		{
			Debug.Log("Socket error: " + e);
            BTNGO.interactable = true;
		}
	}

	public void WriteSocket(string line)
	{
		if (!socket_ready)
			return;

		line = line + "[/TCP]";
		socket_writer.Write(line);
		socket_writer.Flush();
		Debug.Log ("Write:" + line);
	}

	public void CloseSocket()
	{
		if (!socket_ready)
			return;

		socket_writer.Close();
		socket_reader.Close();
		tcp_socket.Close();
		socket_ready = false;
        BTNGO.interactable = true;
	}

    void OnApplicationQuit()
	{
		if(clientSock != null)
			clientSock.Abort ();
		CloseSocket();
	}

    public static void FakeInvoke(){
        OnRecieveGameFinished?.Invoke();
    }
}