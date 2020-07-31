using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.IO;


public class PollerGlove : MonoBehaviour {

    public GameObject Cube;
    public string path = "C:\\Users\\Aviel\\Desktop\\test.txt";
    private GloveAdapter adapter;
    private void Awake()
    {

    }

    // Use this for initialization
    void Start () {
        Cube = GameObject.Find("Cube");
        adapter = Cube.GetComponent<GloveAdapter>();
    }
	// Update is called once per frame
	void Update () {
        float[,] array = new float[3, 3];
        using (StreamWriter sw = File.AppendText(path))
        {
            for (int curr_gyro_entry = 1; curr_gyro_entry <=2; curr_gyro_entry++)
            { 
                if (adapter.getGyroValues(curr_gyro_entry, array))
                {
                    sw.Write("gyro " + curr_gyro_entry + ": \n");
                    Debug.Log("gyro " + curr_gyro_entry + ": \n");
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                        {
                            sw.Write(array[i, j] + " ");
                            Debug.Log(array[i, j] + " ");
                        }
                        sw.WriteLine("");
                        Debug.Log("");
                    }
                } else
                {
                    sw.WriteLine("gyro " + curr_gyro_entry + " not valid yet");
                    Debug.Log("gyro " + curr_gyro_entry + " not valid yet");

                }
            }
            
        }
    }
}
