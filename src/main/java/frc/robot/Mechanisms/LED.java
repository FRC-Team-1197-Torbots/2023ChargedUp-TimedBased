package frc.robot.Mechanisms;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class LED{

    public LED(){
        
    }
    public static enum setLED{
        CONE, CUBE, NONE
    }
    public void run(setLED LEDState){
        byte[] m_ledsignal = {0x00};
        switch(LEDState){
            case CONE:
            m_ledsignal[0] = 0x01;
            case CUBE:
            m_ledsignal[0] = 0x02;
            case NONE:
            m_ledsignal[0] = 0x00;
        }
        int SERVICE_PORT = 8888;

        try{
        DatagramSocket clientSocket = new DatagramSocket();
        
        // Get the IP address of the server
        InetAddress IPAddress = InetAddress.getByName("10.11.97.77");
        
        // Creating a UDP packet 
        DatagramPacket sendingPacket = new DatagramPacket(m_ledsignal, m_ledsignal.length,IPAddress, SERVICE_PORT);
        
        // sending UDP packet to the server
        clientSocket.send(sendingPacket);
        
        // Closing the socket connection with the server
        clientSocket.close();
        }
        catch(IOException e) {
        e.printStackTrace();
        }

    }
}