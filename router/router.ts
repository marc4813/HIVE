const subProc = require("child_process");
const fs = require("fs"); //add input for logging stuff.
import {Scanner} from './scanner';

export class Token{
    name:string; // wlan/eth, etc. VERSION
    data:string;
    id:number;
    freq:number;

    constructor(name:string){
        this.name = name;
        this.data = "";
        this.id = -1;
        this.freq = -1;
    }

    toString = ():string =>{
        return this.name+this.id;
    }
}

export enum ErrorType{ //export this, so ros and websocket can use that.
    sudo = 1,
    os,
    ifconfig,
    hotspotCustom,
    hotspotNet,
    net,
    none
}

export let logError = (error:ErrorType):void =>{
    process.stdout.write("ERROR: This Software Requires ");

    switch(error){
        case ErrorType.sudo:
            console.log("Admininistrative Priveledges");
            break;
        case ErrorType.os:
           console.log("Raspbian Buster Installed.");
           break;
        case ErrorType.ifconfig:
            console.log("ifconfig Installed.");
            break;
        default:
            console.log("Internet Connection");
    }
}

let runCommand = (command:string, options?:string[]):string =>{
    let res:string = "";
    let object:any;

    if(options){
        object = subProc.spawnSync(command, options);
    }
    else{
        object = subProc.spawnSync(command);
    }

    if(object.stdout){
        res = object.stdout.toString();
    }

    return res;
};

let createFreqMap = (tokens:Token[]):Map<string, number> =>{
    let freqMap:Map<string, number> = new Map<string, number>();

    for(let next:number = 0; next < tokens.length; ++next){
        let nextToken:Token = tokens[next];

        if(nextToken.name == "wlan" || nextToken.name == "uap"){
            if(nextToken.freq == 2.4){
                freqMap[nextToken.toString()] = nextToken.freq;
            }
        }
    }

    return freqMap;
}

let createNetMap = (tokens:Token[], freqMap:Map<string, number>):Map<string, Token[]> =>{
    let netMap:Map<string, Token[]> = new Map<string, Token[]>();

    for(let next:number = 0; next < tokens.length; ++next){
        let canAdd:boolean = true;
        let nextToken:Token = tokens[next];

        if(nextToken.data){
            let completeName:string = nextToken.toString();

            if(!netMap.has(nextToken.name)){
                netMap[nextToken.name] = [];
            }

            if(nextToken.name == "wlan" || nextToken.name == "uap"){
                if(freqMap[completeName] != 2.4){ //floor.
                    canAdd = false;
                }
                else{
                    nextToken.freq = freqMap[completeName];
                }
            }

            if(canAdd){
                netMap[nextToken.name].push(nextToken);
            }
        }
    }

    return netMap;
}

let parseOs = (tokens:Token[]):ErrorType =>{
    let error:ErrorType = ErrorType.none;

    if(tokens[0].data != "Raspbian GNU/Linux" || tokens[1].data != "buster"){
        error = ErrorType.os;
    }

    return error;
}

let parseNet = (netMap:Map<string, Token[]>, interName?:string):ErrorType =>{ //load it to hashmap after. Keep method signatures uniform.
    let error:ErrorType = ErrorType.none;
    let command:string = "bash /home/pi/rpihotspot/setup-network.sh"; //fix this 2 separate arguments.

    //turn off uap if it is not 2.4 ghz.

    if(!netMap.has("wlan") || !netMap.has("ether")){
        error = ErrorType.net; //no internet.
    }
    else{
        if(!netMap.has("uap")){
            if(netMap.has("wlan")){
                let wlans:Token[] = netMap["wlan"];
                let searchRes:number = -1;
                let command:string = "";

                //either find by name, or find anything that has 2.4ghz.
                for(let next:number = 0; next < wlans.length; ++next){
                    let nextWlan = wlans[next];
                    let completeName:string = nextWlan.toString();
                    searchRes = next;

                    if(interName){
                        if(completeName == interName){
                            break;
                        }
                    }
                    else{
                        break;
                    }
                }

                if(searchRes == -1){
                    error = ErrorType.hotspot; //maybe add hotspot error types.
                }
                else{
                    runCommand(); //will work.
                }
            }
        }
        else{
            if(!netMap.has("wlan")){ //uap is based on wlan, makes no sense to check ethernet.
                error = ErrorType.net;
            }
        }
    }

    return error;
}

let clean = (netMap:Map<string, Token[]>):void =>{
    if(netMap.has("uap")){
        runCommand("");
    }
}

// add log feature to log installation and other errors. Will ctrl+c kill everything?
let main = ():void =>{
    //let lex = new lexer.Lexer(); //eh?
    let argv:string[] = process.argv;
    let startLog:boolean = false;
    //let interName:string = null;
    let uid:string[] = runCommand("whoami").split('');

    uid.pop();

    if(uid.join('') != "root"){
        logError(ErrorType.sudo);
    }
    else{
        let res:string = runCommand("cat", ["/etc/os-release"]);
        let tokens:Token[] = Scanner.getOsTokens(res.split(''));
        let error:ErrorType = parseOs(tokens);

        if(error != ErrorType.none){
            logError(error);
        }
        else{
            res  = runCommand("ifconfig", ["--version"]);
    
            if(!res && !runCommand("apt", ["install", "net-tools"])){ //fix this.
                logError(ErrorType.ifconfig); //change this error.
            }
            else{
                res = runCommand("iw", ["dev"]);
                tokens = Scanner.getIwTokens(res.split(''));
                let freqMap:Map<string, number> = createFreqMap(tokens);
                res = runCommand("ifconfig");
                tokens = Scanner.getIfTokens(res.split(''));
                let netMap:Map<string, Token[]> = createNetMap(tokens, freqMap);
                error = parseNet(netMap);
    
                if(error != ErrorType.none){
                    logError(ErrorType.net);
                }
                else{
                    // if(startServer(startLog)){
                    //     startRos(startLog);
                    //     //add message
                    // }

                    console.log("ta da!!!!");
                }
            }
        }
    }

    

    // if(argv.length > 2){ //later.
    //     if(argv[2] == "-l"){
    //         startLog = true;
    //     }
        
    //     if(argv.length == 3){
    //         let searchRes:string = argv[3].match(/\w{4}/)[0];
            
    //         if(searchRes == "wlan"){
    //             interName = argv[3];
    //         }
    //     }
    // }
    
}

main();
