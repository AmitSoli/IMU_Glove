using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.IO;



public class GloveAdapter : MonoBehaviour {

    const int NUM_GYROS = 8;
    public GyroData[] gyro_data = new GyroData[NUM_GYROS];
    public SerialPort sp = new SerialPort("COM3", 9600);
    public string path = "C:\\Users\\Aviel\\Desktop\\out.txt";

    // Use this for initialization
    void Start ()
    {
        for (int i = 0; i<NUM_GYROS; i++)
        {
            gyro_data[i].is_valid = false;
            gyro_data[i].asix_vec = new float[9];
        }
        // This text is added only once to the file.
        if (!File.Exists(path))
        {
            // Create a file to write to.
            using (StreamWriter sw = File.CreateText(path))
            {
                sw.WriteLine("Hello");
                sw.WriteLine("And");
                sw.WriteLine("Welcome");
            }
        }
        sp.Open();
        //sp.ReadTimeout = 10;
    }
    void Update()
    {

        byte[] polledBytes = new byte[38];
        if (sp.IsOpen)
        {
            using (StreamWriter sw = File.AppendText(path))
            {
                int bytes_read = 0;
                while (bytes_read != 38)
                {
                    try
                    {
                        int cur_bytes = sp.Read(polledBytes, bytes_read, 38 - bytes_read);
                        bytes_read += cur_bytes;
                        //sw.WriteLine("current_read -" + cur_bytes + ", total = " + bytes_read);
                    }
                    catch (System.Exception e)
                    {
                        sw.WriteLine("bytes read " + bytes_read);
                        sw.WriteLine("Exception*******: " + e.ToString());
                    }
                }
                int index = polledBytes[0] & 7;
                sw.WriteLine("index = " + index + ", start " + polledBytes[0] + ", end = " + polledBytes[37] + ", xor = " + (polledBytes[0] ^ polledBytes[37]));

                if (seq_is_valid(polledBytes[0], polledBytes[37]))
                {
                    sw.Close();
                    update_vec(polledBytes);
                }
                else
                {
                    for (int i = 1; i < 38; i++)
                    {
                        int current_byte = sp.ReadByte();
                        if (seq_is_valid((Byte)current_byte, polledBytes[i]))
                        {
                            sw.WriteLine("seq good after " + i + "\n");
                            break;
                        }
                    }
                }
            }

        }
    }
    bool seq_is_valid(Byte start, Byte end)
    {
        return ((start ^ end) == 0xff);
    }
    public bool getGyroValues(int gyro_index, float[,] gyro_vec)
    {
        if (gyro_index < 0 || gyro_index > 7)
        {
            return false;
        }
        if (gyro_data[gyro_index].is_valid == false)
        {
            return false;
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                gyro_vec[i,j] = gyro_data[gyro_index].asix_vec[i * 3 + j];
            }
        }
        return true;
    }

    void update_vec(Byte[] polledBytes)
    {
        int index = (polledBytes[0] & 7);
        gyro_data[index].is_valid = false;
        //GyroData
        int vec_index = 0;
        for (int i = 1; i < 37; i += 4)
        {
            gyro_data[index].asix_vec[vec_index] = convert_to_float(polledBytes[i], polledBytes[i + 1], polledBytes[i + 2], polledBytes[i + 3]);
            vec_index += 1;
        }
        gyro_data[index].is_valid = true;
    }
    public float convert_to_float(Byte b1, Byte b2, Byte b3, Byte b4)
    {
        bool pos = true;
        
        if (b1 >= 128)
        {
            b1 -= 128;
            pos = false;
        }
        int new1 = b4;
        int new2 = b3 << 8;
        int new3 = b2 << 16;
        int new4 = b1 << 24;
        float abs_num = (new1 | new2 | new3 | new4) / 100000000.0f;
        if (pos)
        {
            return abs_num;
        }
        else
        {
            return -abs_num;
        }
    }

    public struct GyroData
    {
        public float[] asix_vec;
        public bool is_valid;
    }
    
}