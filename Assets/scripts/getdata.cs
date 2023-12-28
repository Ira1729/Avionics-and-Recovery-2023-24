using UnityEngine;
using System;
using System.IO.Ports;
using System.Collections;

public class SerialPortReader : MonoBehaviour
{
    // Set the COM port and baud rate according to your setup
    public string portName = "COM8"; //replace this with the port number
    public int baudRate = 9600; //replace it with the right baud rate
    public SerialPort serialPort;


    void Start()
    {
        serialPort = new SerialPort(portName, baudRate);

        try
        {

            serialPort.Open();


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
            try
            {

                string data = serialPort.ReadLine();


                string[] values = data.Split(',');


                ProcessParsedData(values);

            }
            catch (TimeoutException)
            {
                // Ignore timeout exceptions (thrown when no data is available)
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
        if (values.Length >= 14)
        {

            float x = float.Parse(values[1]);
            float y = float.Parse(values[2]);
            float z = float.Parse(values[3]);
            Debug.Log("Parsed Data: X=" + x + ", Y=" + y + ", Z=" + z);
        }
    }


  
}
