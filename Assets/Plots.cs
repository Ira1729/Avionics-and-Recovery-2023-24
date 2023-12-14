using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class ImageChanger : MonoBehaviour
{
    public RawImage targetImage;
    public string imagePath = "Assets/frame.png"; // Path to the image file
    public float delayBetweenImages = 2f; // Delay in seconds between image changes

    private Texture2D currentTexture;

    void Start()
    {
        StartCoroutine(ChangeImageLoop());
    }

    IEnumerator ChangeImageLoop()
    {
        while (true)
        {
            LoadImage(imagePath);
            yield return new WaitForSeconds(delayBetweenImages);
        }
    }

    void LoadImage(string path)
    {
        if (System.IO.File.Exists(path))
        {
            byte[] fileData = System.IO.File.ReadAllBytes(path);
            currentTexture = new Texture2D(2, 2);
            currentTexture.LoadImage(fileData);

            // Apply the new texture to the RawImage
            targetImage.texture = currentTexture;
        }
        else
        {
            Debug.LogError("Image not found: " + path);
        }
    }
}
