// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// using Microsoft.Azure.Kinect.Sensor;
// using System.Threading.Tasks;

// using System.IO;
// using System;

// public class AtKinectController : MonoBehaviour
// {
//     Device kinect;
//     int num;

//     Mesh mesh;
//     Vector3[] vertices;
//     Color32[] colors;
//     int[] indices;
//     Transformation transformation;

//     public string fileName = "pc.bin";

//     public SignalClient kinectClient;
//     public float sendDelay = 1f;
//     public int everyCount = 1000;
//     public short depthFilterMin = 5000;
//     public short depthFilterMax = 9000;

//     string path;
//     void Start()
//     {
//         InitKinect();
//         InitMesh();

//         Task t = KinectLoop();
//         //t.Start();

//         StartCoroutine(LoopSend());

//         path = Application.dataPath + "/../" + fileName;
//         writer = new StreamWriter(path, true);
        
//         //bw = new BinaryWriter(fs);

//         return;
//     }

//     IEnumerator LoopSend(){
//         while(true){
//             kinectClient.SocketSend(CatchStringData());

//             yield return new WaitForSeconds(sendDelay);
//         }
//     }

//     string CatchStringData(){
//         if (currentArray == null)
//             return null;

//         //Debug.Log("Start Write.");

//         string data = "";

//         int index = 0;
//         int count = 0;
//         bool isFirst = true;
//         foreach (Short3 value in currentArray)
//         {
//             index = (index + UnityEngine.Random.Range(1,2)) % everyCount;
//             if(index % everyCount != 0)
//                 continue;

//             if(value.Z < depthFilterMin || value.Z > depthFilterMax){
//                 continue;
//             }

//             if(isFirst){
//                 data += $"{value.X},{value.Y},{value.Z}";
//                 isFirst = false;
//             }
//             else
//                 data += $"|{value.X},{value.Y},{value.Z}";

//             count++;
//         }
//         //byte[] allByte = menStream.ToArray();
//         //menStream.Read(allByte, 0, allByte.Length);

//         // fs = new FileStream(path, FileMode.OpenOrCreate, FileAccess.Write);
//         // fs.Write(data);
//         // fs.Flush();
//         // fs.Close();
//         //string result = data.Remove(data.Length - 1);
//         //Debug.Log(result);
//         //Debug.Log("Write Finished");

//         //return "123";
//         Debug.Log($"Send count : {count}");
//         return data;
//     }

//     void InitKinect()
//     {
//         kinect = Device.Open(0);

//         kinect.StartCameras(new DeviceConfiguration
//         {
//             ColorFormat = ImageFormat.ColorBGRA32,
//             ColorResolution = ColorResolution.R720p,
//             DepthMode = DepthMode.NFOV_2x2Binned,
//             SynchronizedImagesOnly = true,
//             CameraFPS = FPS.FPS30
//         });

//         transformation = kinect.GetCalibration().CreateTransformation();
//     }

//     void InitMesh()
//     {
//         int width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
//         int height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
//         num = width * height;

//         mesh = new Mesh();
//         mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

//         vertices = new Vector3[num];
//         colors = new Color32[num];
//         indices = new int[num];

//         for (int i = 0; i < num; i++)
//         {
//             indices[i] = i;
//         }

//         mesh.vertices = vertices;
//         mesh.colors32 = colors;
//         mesh.SetIndices(indices, MeshTopology.Points, 0);

//         gameObject.GetComponent<MeshFilter>().mesh = mesh;
//     }

//     Short3[] currentArray;
//     StreamWriter writer;
//     FileStream fs;
//     BinaryWriter bw;

//     public void WriteString(string str)
//     {
//         writer.Write(str);
//         writer.Flush();
//     }

//     public int byteCapacity = 999999;
//     byte[] CatchData()
//     {
//         if (currentArray == null)
//             return null;

//         Debug.Log("Start Write.");

//         MemoryStream menStream = new MemoryStream();

//         foreach (Short3 value in currentArray)
//         {
//             byte[] num = BitConverter.GetBytes(value.X);
//             menStream.Write(num, 0, num.Length);
//             num = BitConverter.GetBytes(value.Y);
//             menStream.Write(num, 0, num.Length);
//             num = BitConverter.GetBytes(value.Z);
//             menStream.Write(num, 0, num.Length);
//         }
//         byte[] allByte = menStream.ToArray();
//        // menStream.Read(allByte, 0, allByte.Length);

//         fs = new FileStream(path, FileMode.OpenOrCreate, FileAccess.Write);
//         fs.Write(allByte, 0 , allByte.Length);
//         fs.Flush();
//         fs.Close();

//         Debug.Log("Write Finished");

//         return allByte;
        
//     }

//     async Task KinectLoop()
//     {
//         while (true)
//         {
//             using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
//             {
//                 Image colorImage = transformation.ColorImageToDepthCamera(capture);

//                 BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();

//                 Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);

//                 Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

//                 currentArray = xyzArray;

//                 float amp = 0.001f;
//                 for (int i = 0; i < num; i++)
//                 {
//                     vertices[i].x = xyzArray[i].X * amp;
//                     vertices[i].y = xyzArray[i].Y * amp;
//                     vertices[i].z = xyzArray[i].Z * amp;

//                     colors[i].b = colorArray[i].B;
//                     colors[i].g = colorArray[i].G;
//                     colors[i].r = colorArray[i].R;
//                     colors[i].a = 255;
//                 }

//                 mesh.vertices = vertices;
//                 mesh.colors32 = colors;
//                 mesh.RecalculateBounds();

//             }
//         }
//     }

//     private void OnDestroy()
//     {
//         if(kinect != null)
//             kinect.StopCameras();
//     }

//     private void OnApplicationQuit() {
//         if(writer != null) writer.Close();
//         if(bw != null) bw.Close();
//         if(fs != null) fs.Close();
//     }
// }
