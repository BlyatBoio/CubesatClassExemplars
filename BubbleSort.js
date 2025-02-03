let sortArray = [];
let sortedArray = [];

function setup() {
  createCanvas(800, 150);

  // minimum value, maximum value, amount of values
  sortArray = fillRandomArray(0, 10, 20); // define the arary to be sorted as a bunch of random values
  sortedArray = bubbleSort(sortArray); // sort the array and store it as sorted array

  // formatting
  background(100);
  fill(255);
  stroke(0);

  // display both arrays for comparison
  text("Unsorted Array: " + arrayToString(sortArray), 10, 50);
  text("Sorted Array: " + arrayToString(sortedArray), 10, 100);
}

function bubbleSort(array){ 
  // ensure we dont change the input array by copying it to a different variable
  // arrayCopy function exists but for whatever reason, it was giving me errors
  // this itterates over all of the array values, and pushed them to a new empty array "newArray"
  let newArray = [];

  for(let i = 0; i < array.length; i++){
    newArray.push(array[i]); 
  }

  let isSorted = false; // boolean to exit the while loop once sorting is done

  // ensure that the the loop doesent run on forever
  let maxItterations = 100;
  let itterations = 0;

  while(isSorted == false){
    itterations ++;

    let numbersSwapped = 0; // numbers swapped totals the amount of times a number was out of place in the array
    // once this number is 0, we can exit the while loop and ensure that the array is sorted
    
    for(let i = 0; i < newArray.length; i++){

      // if the given number is greater than the index above it
      // or if in this case [8, 4] the two are swapped, in this case [8, 4] they are not
      if(newArray[i] > newArray[i+1]){
        // because we are directly setting the values, we cant simultaneously set the values
        // and we need to save one to apply later
        let saveValue = newArray[i+1];

        newArray[i+1] = newArray[i]; // update the value we saved
        newArray[i] = saveValue; // set the other value to the value we saved
        numbersSwapped ++; // incriment the numbers swapped value so the program knows the array may not be sorted yet
      }
    }
    if(numbersSwapped == 0) isSorted = true; // if no numbers were swapped, exit the loop because the array was sorted

    if(itterations > maxItterations) break; // break exits the while loop without satisfying the condition, sort may have failed
  } 
  return newArray; // return the sorted array
}

function fillRandomArray(minValue, maxValue, amntValues){
  let newArray = []; // define an empty array

  // itterate over the given ammount of values
  for(let i = 0; i < amntValues; i++){
    // if you dont round the values, they will have a lot of decimals and not be as clean
    newArray.push(round(random(minValue, maxValue))); // push the random values
  }

  return newArray; // return the array
}

function arrayToString(array){
  let arrayString = "";// define the variable for the final string

  // itterate over the array
  for(let i = 0; i < array.length; i++){

    arrayString += array[i]; // add the value at the current index
    
    if(i != array.length-1) arrayString += ", "; // if it is not at the end of the array, add a ", " in between values
  }
  // display the final string at the provided x and y location
  return arrayString;
}
