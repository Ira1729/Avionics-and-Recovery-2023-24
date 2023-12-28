using UnityEngine;
using System;
using System.IO.Ports;
using System.Collections;
public class ChangeEulerAngles : MonoBehaviour
{
    // Set the desired Euler angles
    public Vector3 targetRotation = new Vector3(0f, 90f, 0f);
 
    public string portName = "COM4"; //replace this with the port number
    public int baudRate = 9600; //replace it with the right baud rate
    public SerialPort serialPort;


    void Start()
    {
        serialPort = new SerialPort(portName, baudRate);

        try
        {

            serialPort.Open();
            Debug.Log("Port Opened");


            StartCoroutine(ReadSerialData(serialPort));
        }
        catch (Exception e)
        {
            Debug.LogError("Error opening serial port: " + e.Message);
        }
    }


    IEnumerator ReadSerialData(SerialPort serialPort)
    {
        
        
        while (true)
        {
            yield return new WaitForSeconds(1.0f);
            try
            {
                

                string data = serialPort.ReadLine();
                Debug.Log("Data read Succesfully");


                string[] values = data.Split(',');


                ProcessParsedData(values);
                Debug.Log("Data sent to transform");

            }
            
            catch (TimeoutException)
            {
                Debug.LogError("Timeout while reading data");// Ignore timeout exceptions (thrown when no data is available)
            }
            catch (Exception e)
            {
                Debug.LogError("Error reading serial data: " + e.Message);
                yield break;
            }
        }
        
    }
    void OnDestroy()
    {

        serialPort.Close();
    }

    void ProcessParsedData(string[] values)
    {
     

            float x = float.Parse(values[1]);
            float y = float.Parse(values[2]);
            float z = float.Parse(values[3]);
            Debug.Log("Parsed Data: X=" + x + ", Y=" + y + ", Z=" + z);
            transform.eulerAngles = new Vector3(x-90,y,z);
     
    }

}
