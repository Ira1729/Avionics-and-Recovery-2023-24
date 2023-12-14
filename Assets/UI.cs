using UnityEngine;
using UnityEngine.UI;

public class Graph : MonoBehaviour
{
    public Image graphImage;
    public float updateInterval = 0.1f; // Set the update interval in seconds
    public int maxDataPoints = 100; // Set the maximum number of data points to display

    private float timer = 0f;
    private RectTransform graphRect;
    private Vector2[] dataPoints;

    void Start()
    {
        if (graphImage == null)
        {
            Debug.LogError("Image not assigned!");
            return;
        }

        graphRect = graphImage.rectTransform;

        // Initialize the data points array
        dataPoints = new Vector2[maxDataPoints];
    }

    void Update()
    {
        timer += Time.deltaTime;

        // Update the plot at the specified interval
        if (timer >= updateInterval)
        {
            UpdatePlot();
            timer = 0f;
        }
    }

    void UpdatePlot()
    {
        // Simulate real-time data (replace this with your data source)
        float newDataPoint = Random.Range(0f, 1f);

        // Shift the existing data points to the left
        for (int i = 0; i < dataPoints.Length - 1; i++)
        {
            dataPoints[i] = new Vector2(i + 1, dataPoints[i + 1].y);
        }

        // Add a new data point at the end
        dataPoints[dataPoints.Length - 1] = new Vector2(maxDataPoints - 1, newDataPoint);

        // Draw the graph
        DrawGraph();
    }

    void DrawGraph()
    {
        // Create a texture to draw the graph
        Texture2D texture = new Texture2D((int)graphRect.rect.width, (int)graphRect.rect.height);
        Color[] colors = new Color[texture.width * texture.height];
        for (int i = 0; i < colors.Length; i++)
    {
        colors[i] = Color.black;
    }

        // Draw the data points on the texture
        for (int i = 0; i < dataPoints.Length; i++)
        {
            int x = Mathf.RoundToInt(dataPoints[i].x);
            int y = Mathf.RoundToInt(dataPoints[i].y * texture.height);

            // Ensure the coordinates are within the texture bounds
            x = Mathf.Clamp(x, 0, texture.width - 1);
            y = Mathf.Clamp(y, 0, texture.height - 1);

            colors[y * texture.width + x] = Color.white;
        }

        // Apply the colors to the texture
        texture.SetPixels(colors);
        texture.Apply();

        // Apply the texture to the UI Image
        Sprite sprite = Sprite.Create(texture, new Rect(0, 0, texture.width, texture.height), Vector2.zero);
        graphImage.sprite = sprite;
    }
}
