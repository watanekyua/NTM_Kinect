using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Microsoft.Azure.Kinect.Sensor;
using System.Threading.Tasks;

using System.IO;


public class AtKinectController : MonoBehaviour
{
    Device kinect;
    int num;

    Mesh mesh;
    Vector3[] vertices;
    Color32[] colors;
    int[] indices;
    Transformation transformation;

    public string fileName = "pc.bin";

    string path;
    void Start()
    {
        InitKinect();
        InitMesh();

        Task t = KinectLoop();
        t.Start();

        return;

        path = Application.dataPath + "/../" + fileName;
        //writer = new StreamWriter(path, true);
        fs = new FileStream(path, FileMode.OpenOrCreate, FileAccess.Write);
        bw = new BinaryWriter(fs);
    }

    void InitKinect()
    {
        kinect = Device.Open(0);

        kinect.StartCameras(new DeviceConfiguration
        {
            ColorFormat = ImageFormat.ColorBGRA32,
            ColorResolution = ColorResolution.R720p,
            DepthMode = DepthMode.NFOV_2x2Binned,
            SynchronizedImagesOnly = true,
            CameraFPS = FPS.FPS30
        });

        transformation = kinect.GetCalibration().CreateTransformation();
    }

    void InitMesh()
    {
        int width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
        int height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
        num = width * height;

        mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        vertices = new Vector3[num];
        colors = new Color32[num];
        indices = new int[num];

        for (int i = 0; i < num; i++)
        {
            indices[i] = i;
        }

        mesh.vertices = vertices;
        mesh.colors32 = colors;
        mesh.SetIndices(indices, MeshTopology.Points, 0);

        gameObject.GetComponent<MeshFilter>().mesh = mesh;
    }

    Short3[] currentArray;
    StreamWriter writer;
    FileStream fs;
    BinaryWriter bw;

    [ContextMenu("Catch Data")]
    void CatchData()
    {
        if (currentArray == null)
            return;

        // string result = "";

        // for (int i = 0; i < currentArray.Length; i++)
        // {
        //     result = $"{currentArray[i].X},{currentArray[i].Y},{currentArray[i].Z}|";
        //     WriteString(result);
        // }

        // result += "|";

        Debug.Log("Start Write.");
        WriteShorts(currentArray);
        Debug.Log("Write Finished");
        //Debug.Log(result);
    }

    public void WriteString(string str)
    {
        writer.Write(str);
        writer.Flush();
    }

    private void OnApplicationQuit() {
        if(writer != null) writer.Close();
        if(bw != null) bw.Close();
        if(fs != null) fs.Close();
    }

    void WriteShorts(Short3[] values)
    {
        foreach (Short3 value in values)
        {
            bw.Write(value.X);
            bw.Write(value.Y);
            bw.Write(value.Z);
        }
        bw.Flush();
    }

    async Task KinectLoop()
    {
        while (true)
        {
            using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
            {
                Image colorImage = transformation.ColorImageToDepthCamera(capture);

                BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();

                Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);

                Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                currentArray = xyzArray;

                float amp = 0.001f;
                for (int i = 0; i < num; i++)
                {
                    vertices[i].x = xyzArray[i].X * amp;
                    vertices[i].y = xyzArray[i].Y * amp;
                    vertices[i].z = xyzArray[i].Z * amp;

                    colors[i].b = colorArray[i].B;
                    colors[i].g = colorArray[i].G;
                    colors[i].r = colorArray[i].R;
                    colors[i].a = 255;
                }

                mesh.vertices = vertices;
                mesh.colors32 = colors;
                mesh.RecalculateBounds();

            }
        }
    }

    private void OnDestroy()
    {
        if(kinect != null)
            kinect.StopCameras();
    }
}
