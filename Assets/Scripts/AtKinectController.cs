// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// using Microsoft.Azure.Kinect.Sensor;

// public class AtKinectController : MonoBehaviour
// {
//     Device kinect;

//     Texture2D kinectColorTexture;

//     [SerializeField]
//     UnityEngine.UI.RawImage rawColorImg;

//     void Start(){
//         InitKinect();
//     }

//     void InitKinect(){
//         kinect = Device.Open(0);

//         kinect.StartCameras(new DeviceConfiguration{
//             ColorFormat = ImageFormat.ColorBGRA32,
//             ColorResolution = ColorResolution.R720p,
//             DepthMode = DepthMode.NFOV_2x2Binned,
//             SynchronizedImagesOnly = true,
//             CameraFPS = FPS.FPS30
//         });

//         int width = kinect.GetCalibration().ColorCameraCalibration.ResolutionWidth;
//         int height = kinect.GetCalibration().ColorCameraCalibration.ResolutionHeight;

//         kinectColorTexture = new Texture2D(width, height);
//     }

//     void Update(){
//         Capture capture = kinect.GetCapture();

//         Image colorImage = capture.Color;

//         Color32[] pixels = colorImage.GetPixels<Color32>().ToArray();

//         for (int i = 0; i < pixels.Length; i++)
//         {
//             var d = pixels[i].b;
//             var k = pixels[i].r;
//             pixels[i].r = d;
//             pixels[i].b = k;
//         }

//         kinectColorTexture.SetPixels32(pixels);
//         kinectColorTexture.Apply();

//         rawColorImg.texture = kinectColorTexture;

//         capture.Dispose();
//     }

//     private void OnDestroy() {
//         kinect.StopCameras();
//     }
// }
