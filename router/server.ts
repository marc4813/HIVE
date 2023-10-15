const fs = require("fs");
const ws = require("ws");
const roslib = require("roslib");
import { WebSocketServer } from "ws";

enum MessageTypes{
    geometry = 1,
    laserscan
};

export let startServer = (logFile:string):void =>{
    const agents = new Array();
    let servRef:WebSocketServer;

    const ros = new roslib.Ros({
        url:"ws://144.126.249.86:9090"
    });

    ros.on("connection", ()=>{
        console.log("SUCCESS: Connected to the Robotic Backend");

        try{
            const server = new ws.Server({port:80});
            servRef = server;

            console.log("SUCCESS: Started the Web Socket Server on Port 80");
    
            server.on("connection", (agent)=>{
                agent.id = agents.length+1;
        
                agents.push(agent);
        
                console.log(`WS: Agent ${agent.id} Connected`);

                const cmdVel = new roslib.Topic({
                    ros: ros,
                    name: "/agent"+agent.id+"/cmd_vel",
                    messageType: "geometry_msgs/Twist"
                });
        
                const laserScan = new roslib.Topic({
                    ros: ros,
                    name: "/agent"+agent.id+"/laserscan",
                    messageType: "sensor_msgs/Laserscan"
                });

                cmdVel.subscribe((data)=>{
                    console.log(`SUBSCRIBED: ${data}`);

                    console.log(data);
        
                    const message = {
                        type: MessageTypes.geometry,
                        data: {
                            linear: {
                                x: data.linear.x,
                                y: data.linear.y
                            },
                            angular:{
                                z: data.angular.z
                            }
                        }
                    }
        
                    agent.send(JSON.stringify(message));
                });
        
                agent.on("message", (data)=>{
                    const parsed = JSON.parse(data);
        
                    console.log(`MESSAGE: ${parsed.data} from Agent ${agent.id}`);
        
                    switch(parsed.type){
                        case MessageTypes.laserscan: //fix this. Will there be more?
                            const data = parsed.data;
                            const header = data.header;
                            let seqId:number = header[0];
                            let frameId:string = header[1];
                            let numPts:number = header[2];
                            let ranges:number[] = [];
                            let intensities:number[] = [];
                            let flags:number[] = [];
    
                            for(let next:number = 0; next < numPts; ++next){
                                intensities.push(data.intensity[next]);
                                ranges.push(data.distance[next]);
                                flags.push(data.flag[next]);
                            }

                            const message = {
                                "header":{
                                    "stamp":{
                                        "secs": 0,
                                        "nsecs": 0
                                    },
                                    "seq": seqId,
                                    "frame_id": frameId
                                },
                                "angle_min":  -6.28318977355957,
                                "angle_max": 6.28318977355957,
                                "angle_increment": 0.54,
                                "scan_time": 0.0,
                                "time_increment": 0.0,
                                "range_min": 0.07999999821186066,
                                "range_max": 8.0,
                                "ranges": ranges,
                                "intensities": intensities
                            }
        
                            laserScan.publish(message);
        
                            console.log(`PUBLISHED: ${message} to Agent ${agent.id}`);

                            break;
                            
                        default:
                            console.log("unkown message type"); //remove when deployed.
                    }
                });
        
                agent.on("close", (code, data)=>{
                    console.log(code);
                    console.log(`WS: Agent ${agent.id} Disconnected`);
        
                    agents.splice(agent.id-1, 1);
                });

                agent.on("error", ()=>{
                    console.log("no");
                })
            });
        }
        catch(error){
            console.log("ERROR: Could not start the Web Socket Server");

            if(logFile){
                fs.writeFile(logFile, error, null);
            }
        }
    });

    ros.on("error", (error:string)=>{
        console.log("ERROR: Lost Connection to the Robotic Backend");

        if(logFile){
            fs.writeFile(logFile, error, null);
        }
    });

    ros.on("close", ()=>{
        if(servRef){
            servRef.close();
        }
    });
}