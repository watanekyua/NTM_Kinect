using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RenderTextureToRenderTexture : MonoBehaviour
{
    public RenderTexture from;
    public RenderTexture dist;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Graphics.Blit(from, dist);
    }
}
