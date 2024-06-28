/* //////////////// Ramp response measure ////////////////

The script will perform an up and down ramp function with values between "minVal" and "maxVal". Around each ramp the script will keep the output constant (plateau). The ramp will be of duration "t". Data will be continuously recorded during the test.

 ^ Motor Input
 |           _____maxVal   
 |          |     |                            
 |          |     |          
 | minVal___|     |___            
 |_________ _____________> Time

///////////// User defined variables //////////// */

var minVal = 1000;         // Min. input value [700us, 2300us] 
var maxVal = 1500;         // Max. input value [700us, 2300us]
var t = 10;                // Time of the ramp input (seconds)
var plateauDuration = 5;   // Time to wait at each plateau (seconds)
var samplesAvg = 1;       // Number of samples to average at each reading (reduces noise and number of CSV rows)
var repeat = 1;       // How many times to repeat the same sequence
var filePrefix = "myStepTest_onestep";
var rampGoDown = true;      // If set to true, the ramp will go up and down.
var stepsQty = 1;

///////////////// Beginning of the script //////////////////

//Reading sensors and writing to file continuously
rcb.files.newLogFile({Thrust (kgf), ESC signal (µs), Time (s), prefix: filePrefix});
readSensor();   // Start the loop. After readSensor(), readDone() followed by readSensor(), etc.

function readSensor(){
    rcb.console.setVerbose(false);
    rcb.sensors.read(readDone, samplesAvg);
    rcb.console.setVerbose(true);
}

function readDone(result){
    //rcb.console.setVerbose(false);
    rcb.files.newLogEntry(result,readSensor);
    //rcb.console.setVerbose(true);
}

//ESC initialization
rcb.console.print("Initializing ESC...");
rcb.output.set("esc",1000);
rcb.wait(startPlateau, 4);

//Start plateau
function startPlateau(){
    rcb.console.print("Start Plateau...");
    rcb.output.set("esc",minVal);
    rcb.wait(stepUp, plateauDuration);
}

//Ramp up
function stepUp(){
    rcb.console.print("stepping Up...");
    rcb.output.steps("esc", minVal, maxVal, stepsQty, upPlateau); //no optional fct
}

//Up Plateau
function upPlateau() {
    rcb.console.print("Up Plateau...");
    rcb.wait(stepDown, plateauDuration);
}

//Ramp down
function stepDown() {
    if(rampGoDown){
        rcb.console.print("stepping Down...");
        rcb.output.steps("esc", maxVal, minVal, stepsQty, endPlateau);
    }else
        endScript();
}

//End Plateau
function endPlateau() {
    rcb.console.print("End Plateau...");
    rcb.wait(endScript, plateauDuration);
}

//Ends or loops the script
function endScript() {
    if(--repeat > 0){
      if(repeat === 0){
        rcb.console.print("Repeating one last time...");
      }else{
        rcb.console.print("Repeating " + repeat + " more times...");
      }
      startPlateau();
    }else{
      rcb.endScript();
    }
}


    
    



