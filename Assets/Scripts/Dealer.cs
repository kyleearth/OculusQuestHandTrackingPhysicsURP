using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using UnityEngine.Analytics;



public class Dealer : MonoBehaviour
{
    private bool _dealerCancelled = false;
    private IEnumerator _dealerWorkerRoutine;
    public ConcurrentQueue<string> OutBox = new ConcurrentQueue<string>();
    public ConcurrentQueue<string> Inbox = new ConcurrentQueue<string>();
    public List<double> pings = new List<double>();
    private DealerSocket _sock;

    private float interBeatInterval = 5f;
    public delegate void SocketDel();

    private void Start()
    {
        var connections = new List<string>() {$"tcp://40.117.225.101:8000"};
        _dealerWorkerRoutine = DealerWorker(connections);
        StartCoroutine(_dealerWorkerRoutine);
        pings.Clear();
        Debug.Log($"Identity: {UserAndSessionId()}");
    }

    private IEnumerator DealerWorker(List<string> toConnect)
    {
        //must be here for Windows to work. not needed on Mac
        AsyncIO.ForceDotNet.Force();
        
        _sock = new DealerSocket();
        
        _sock.Options.Identity =
            Encoding.Unicode.GetBytes(UserAndSessionId());
        _sock.Options.SendHighWatermark = 1000;
        foreach (var x in toConnect)
        {
            _sock.Connect(x);
        }


        using (var poller = new NetMQPoller {_sock})
        {
            _sock.ReceiveReady += Client_ReceiveReady;
            poller.RunAsync();
        
            while (!_dealerCancelled)
            {
                // handle outgoing
                while (OutBox.Count > 0)
                {
                    if (OutBox.TryDequeue(out string msg))
                    {
                        _sock.SendFrame(msg);
                    }
                }

                _sock.SendFrame($"HEARTBEAT|{BTools.UnixTimeLong()}");
            
                yield return new WaitForSeconds(interBeatInterval);
            }
        }
        NetMQConfig.Cleanup();
        Debug.Log("post");
    }

    private void Client_ReceiveReady(object sender, NetMQSocketEventArgs e)
    {
        while (e.IsReadyToReceive)
        {
            if (e.Socket.TryReceiveFrameString(out var message))
            {
                if (DealerHandleIngress(message, ref pings))
                {
                    Inbox.Enqueue(message);
                }
            }
        }
    }

    private static string UserAndSessionId()
    {
        return "patient_a";
//        return SystemInfo.deviceUniqueIdentifier +"_"+  AnalyticsSessionInfo.sessionId;
    }

    // will handle the incoming message with the special 'HEARTBEAT'.
    // Returns true if the message is not a special and should be enqueued to the list.
    // returns false if the message is a heartbeat throwaway.
    private static bool DealerHandleIngress(string message, ref List<double> pingList)
    {
        if (message.StartsWith("HEARTBEAT"))
        {
            var splits = message.Split('|').Skip(1).Select(Convert.ToDouble).ToArray();
            var xtime = BTools.UnixTimeLong();
            pingList.Add( xtime - (long)splits[0]);
            while (pingList.Count > 10)
            {
                pingList.RemoveAt(0);
            }

            return false;
        }
        
        if (message.Length == 0)
        {
            Debug.Log("Got empty topic.");
            return false;
        } 
        return true;
    }


    private void OnApplicationQuit()
    {
        _dealerCancelled = true;
        Thread.Sleep(100);
        NetMQConfig.Cleanup(false);
//give it a moment
        Thread.Sleep(10);
    }

    public float MeanPing()
    {
        return (float) (pings.Sum() / (pings.Count + 0.0000001f));
    }

    public void SendResponseToBackend(string topic, string payload)
    {
        OutBox.Enqueue($"{topic}|{payload}");
    }
}