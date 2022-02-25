using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JpgToRenderTexture : MonoBehaviour
{
    public RenderTexture rt;
    public Texture source;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         Graphics.Blit(source, rt);
    }
}
