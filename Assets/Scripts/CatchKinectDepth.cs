using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;

public class CatchKinectDepth : MonoBehaviour
{   
    public int sensorIndex = 0;
    public RenderTexture DisplayRender;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        KinectManager kinectManager = KinectManager.Instance;
        if (kinectManager && kinectManager.IsInitialized())
        {
            Texture tex = kinectManager.GetDepthImageTex(sensorIndex);
            Graphics.Blit(tex, DisplayRender);
        }
    }
}
