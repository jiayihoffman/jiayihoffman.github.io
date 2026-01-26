---
layout: post
title: "MapTools iOS App User Guide"
date: 2025-04-05 11:45:28 -0600
categories: MapTools
---

MapTools is a versatile utility app designed to help you work with geographic coordinates easily and accurately. Whether you’re a field surveyor, GIS analyst, hiker, or simply curious about locations, MapTools provides you with the tools to:
* Convert between different coordinate systems, datums, and formats
* Measure distances and bearings between two geographic points
* Preview locations on a map using your current position or manually entered coordinates

MapTools is available from the [App Store](https://apps.apple.com/us/app/map-tools/id329231582).

## Why Different Coordinate Systems and Datums

The Earth isn’t a perfect sphere; it’s slightly flattened at the poles and bulged at the equator. Because of this, multiple coordinate systems and datums have been developed to represent locations more accurately, depending on the region or purpose.
* A coordinate system defines how positions are expressed, such as latitude/longitude (Geodetic) or grid-based (MGRS, GEOREF, UTM).
* A datum is a mathematical model of the Earth used to align the coordinate system with the real world (e.g., WGS84, NAD83, NAD27).

Different applications and regions may utilize various combinations of these, making it essential to convert and compare them easily — and that’s exactly what the MapTools helps you do.

The following coordinate systems are available for selection as either the source or target coordinate system:

* Geodetic (latitude and longitude)
* MGRS
* Georef
* USNG
* UTM/UPS
* Map Grid Australia (MGA) with GDA94 Datum
* British National Grid with OSGB36 Datums
* New Zealand Transverse Mercator 2000 (NZTM2000) with NZGD Datum
* New Zealand Map Grid (NZMG) with NZGD49 Datum
* Swiss Grid with CH1903 Datum
* Greek Grid with EGSA87 Datum
* Singapore TM with SVY21 Datum

For a complete list of datums supported by MapTools, please refer to this [file](/assets/all_datums.txt).

## MapTools Features
### Coordinates Conversion
The Coordinates Conversion feature allows you to convert coordinates among different formats, coordinate systems, and datums. You can input coordinates manually or use your current location.

#### Choose the Input Method
At the top, select your preferred input method for the location:
* Enter Input Location: Manually type in coordinates.
* Use Current Location: Automatically detect your current position using GPS. 

#### Enter or Fetch the Location

✏️ If you chose “Enter Input Location”:
* Select the Coordinate System (e.g., Geodetic, MGRS, GEOREF).
* Select the Datum (e.g., WGS84, NAD27).
* Select the Format (e.g., Decimal Degrees).
* Enter the location text (coordinates) into the input field that matches the example format. 

<a href="/assets/maptools/IMG_2999.PNG" >
  <img src="/assets/maptools/IMG_2999.PNG" width="350" />
</a>
<a href="/assets/maptools/IMG_3001.PNG" >
  <img src="/assets/maptools/IMG_3001.PNG" width="350"/>
</a> 

📍 If you chose “Use Current Location”:
* Tap the “Use Current Location” button.
* Wait while the app fetches your current location.
* Your coordinates will be automatically filled in the input field once available. The "Use Current Location" button is temporarily greyed out while fetching the location. * You can copy the detected location using the copy icon.

Permissions Note:

To use the current location, the app needs location permission. If denied, grant access in your device’s Settings > Privacy > Location Services.

<a href="/assets/maptools/IMG_3059.PNG" >
  <img src="/assets/maptools/IMG_3059.PNG" width="350" />
</a>
<a href="/assets/maptools/IMG_3006.PNG" >
  <img src="/assets/maptools/IMG_3006.PNG" width="350" />
</a>

For either input method, you can use the map icon to preview the location on the map. Tap the map icon next to the input header to visually confirm if your location is correct.

<a href="/assets/maptools/IMG_0583.PNG" >
  <img src="/assets/maptools/IMG_0583.PNG" width="500" />
</a>

#### Choose the Output System 

In the Output section, select your desired Coordinate System, Datum, and Format.

#### Convert the Coordinates

* Tap the Convert button.
* The app will process your input and display the converted location in the output section.

If there is an error (e.g., invalid input or location permissions), it will be shown in red. 

#### View and Copy Output

In the Output section:
*	The converted location will be displayed.
*	Tap the copy icon to copy the output to your clipboard. 
* A green checkmark will briefly appear to confirm it was copied.

### Distance Measure
The Distance Measure feature calculates the distance and bearing between two geographic locations. You can enter the locations manually or utilize your current location.

#### Set Input Locations

✏️ Option A: Manually Enter Coordinates
* Select the Coordinate System, Datum, and Format from the dropdown menus.
* Input coordinates in the Location 1 and Location 2 fields.

📍  Option B: Use Your Current Location
* Click the location icon next to either location field.
* The app will automatically fill in your current location (ensure location permissions are enabled).

Optional: Preview on Map
* Tap the map icon in the Input section to preview your start and end locations on a map.
* The map is available when both inputs are populated with valid coordinates.

#### Choose Distance Units 
Select your preferred distance unit (e.g., kilometers, miles) from the Distance Unit dropdown.

<a href="/assets/maptools/IMG_2994.PNG" >
  <img src="/assets/maptools/IMG_2994.PNG" width="350" />
</a>
<a href="/assets/maptools/IMG_3007.PNG" >
  <img src="/assets/maptools/IMG_3007.PNG" width="350"/>
</a> 

#### Calculate Distance and Bearing
* Tap the Calculate button.
* The app will compute the distance and direction (bearing) between the two points.

If coordinates are missing, invalid, or being fetched, the app will disable certain buttons and display a red error message if necessary.

#### View and Copy Results
* View the Distance and Bearing in the Output section.
* Tap the copy icon next to either field to copy the result to your clipboard.
* A green checkmark will briefly appear to confirm that it was copied.

⸻

💡 Tips
* Switch coordinate systems and formats anytime — the app will automatically clear outdated results.
* You can dismiss the keyboard by tapping “Done” or outside the input field.


If you have any questions or feedback, please feel free to drop us a line. We can be reached at: contact@modularmachines.ai

Cheers!