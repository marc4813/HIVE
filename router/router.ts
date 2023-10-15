const subProc = require("child_process");
const fs = require("fs"); //add input for logging stuff.
const Scanner = require('./scanner');
const startServer = require("./server");

export class Token{
    name:string; // wlan/eth, etc. VERSION
    data:string;
    id:number;
    freq:number;

    constructor(name:string){
        this.name = name;
        this.data = ""; //will it be initialized by default.
        this.id = -1;
        this.freq = -1;
    }

    toString = ():string =>{
        return this.name+this.id;
    }
}

export enum ErrorType{ //export this, so ros and websocket can use that.
    sudo = 1,
    log,
    os,
    ifconfig,
    net,
    hotspot,
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
        case ErrorType.net:
            console.log("Internet Connection");
            break;
        default:
            console.log("2.4 GHz Hotspot. Download it from https://github.com/idev1/rpihotspot");
    }
}

let runCommand = (command:string, options?:string[], logFile?:string):string =>{
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
    else{
        if(logFile){
            fs.writeFile(logFile, object.error, null);
        }
    }

    return res;
};

let createFreqMap = (tokens:Token[]):Map<string, boolean> =>{
    let freqMap:Map<string, boolean> = new Map<string, boolean>();

    for(let next:number = 0; next < tokens.length; ++next){
        let nextToken:Token = tokens[next];

        if(nextToken.name == "wlan" || nextToken.name == "uap"){
            if(nextToken.freq == 2.4){
                freqMap[nextToken.toString()] = true;
            }
        }
    }

    return freqMap;
}

let createNetMap = (tokens:Token[], freqMap:Map<string, boolean>):Map<string, Token[]> =>{
    let netMap:Map<string, Token[]> = new Map<string, Token[]>();

    for(let next:number = 0; next < tokens.length; ++next){
        let nextToken:Token = tokens[next];

        if(nextToken.data){
            if(freqMap[nextToken.toString()]){
                if(!netMap.has(nextToken.name)){
                    netMap[nextToken.name] = new Array();
                }

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

let parseNet = (netMap:Map<string, Token[]>):ErrorType =>{ //load it to hashmap after. Keep method signatures uniform.
    let error:ErrorType = ErrorType.none;

    if(!netMap["wlan"]){
        error = ErrorType.net;
    }
    else{
        if(!netMap["uap"]){
            error = ErrorType.hotspot;
        }
    }

    return error;
}

let main = ():void =>{
    let uid:string[];
    let error:ErrorType = ErrorType.none;
    let logFile:string = "";

    if(process.argv.length == 3 && process.argv[3] == "-l"){
        logFile = "RouterLog.txt";
    }

    uid = runCommand("whoami").split("");

    uid.pop();

    if(uid.join("") != "root"){
        error = ErrorType.sudo;
    }
    else{
        let res:string = runCommand("cat", ["/etc/os-release"]);
        let tokens:Token[] = Scanner.getOsTokens(res.split(''));
        error = parseOs(tokens);

        if(error == ErrorType.none){
            res  = runCommand("ifconfig", ["--version"]);
        
            if(!res){
                error = ErrorType.ifconfig;
            }
            else{
                res = runCommand("iw", ["dev"]);
                tokens = Scanner.getFreqTokens(res.split(""));
                let freqMap:Map<string, boolean> = createFreqMap(tokens);

                res = runCommand("ifconfig");
                tokens = Scanner.getNetTokens(res.split(""));
                let netMap:Map<string, Token[]> = createNetMap(tokens, freqMap);
                error = parseNet(netMap);
        
                if(error == ErrorType.none){
                    startServer(logFile);
                }
            }
        }
    }

    if(error != ErrorType.none){
        logError(error);
    }
}

main();