# All Things RC / neckbeard-laser
An Inkscape plugin for exporting vector and image data as G-code for DIY laser cutters, based on TurnkeyTyranny's et al. code.

#### Compatibility:
You will also need to use my version of Marlin for the base64 raster to work properly.
https://github.com/mayhem2408/buildlog-lasercutter-marlin
In addition you will need a program that sends the G-code file to the laser cutter, like Repetier-Host, LaserWeb or Pronterface.

## Origins & Changes

This plugin is based on TurnkeyTyranny's and ajfouls's (and others) work, check it out here:
[TurnkeyTyranny/laser-gcode-exporter-inkscape-plugin](https://github.com/TurnkeyTyranny/laser-gcode-exporter-inkscape-plugin)

#### I have made the following improvements:

1. In the Inkscape plugin, things were moved around a bit, namely the decisions about what and how to cut.
2. parse_layer_name(): Layer naming now works a bit differently and was fool-proofed, and only visible layers are converted into G-code (WYSIWYG)
3. feedratemod(): Feedrate is lowered when farther away from the 0,0 point in order to compensate for lost laser power (TODO: Should be made optional, and long moves don't really benefit from this. Should be performed by the Arduino.)
4. Rasterbase64(): Modified the original rastering method so that white space is ignored. Each horizontal row of consecutive non-white pixels gets a separate M649 & G7 command (TODO: Not quite ready yet)
5. Raster(): Added a rastering method that breaks images into G1 moves. Color depth can be varied to reduce file size. Consecutive pixels of similar pixels are condensed into a single command. This works on GRBL.
6. Added the repeat layer parameter; The layer can be cut multiple times to hopefully minimize burnt edges.

## Usage
Like in TurnkeyTyranny's original plugin, each layer can have different settings for the laser, simply specified in the layer's name. Layer visibility now decides whether a layer is exported or not (the layer's style is searched for "display:none", this happens in effect_curve()). You can show the Layers menu in Inkscape by pressing shift+ctrl+L.

#### Layer parameters
Layer parameters should be separated by a comma. Whitespace and case doesn't matter. For example: 
```
power=10, feed=3000, repeat=2 
maxpower=50, minpower=10, dir=v, feed=800, raster, resolution=10
```
raster - This layer will be rastered using the method specified in the options (TODO: a layer parameter for each raster method)

power=20 - Specifies the laser power to use for this layer. For raster mode, this sets the maximum laser intensity.

maxpower=50 - Specifies the maximum laser power to use for this laser. Only for base64 raster layers

minpower=5 - Specifies the minimum laser power to use for this laser. Only for base64 raster layers

feed=3000 - The speed at which to move on this layer while the laser is on.

dir=v - Specifies the direction of the base64 raster lines. h = Horizontal and is the default. v = Vertical and 45 = 45 degree diagonal. All 3 options only work with the mayhem2408 version of the marlin firmware.

repeat=2 - How many times to cut this layer (repeat:1 is default and means the layer is cut once)

ppm=60 - Pulses per minute

#### Exporting
In Inkscape, select Extensions > Laser Tools > All Things RC Laser.

Under Preferences, select where to output the G-code file. Fiddle with settings depending on what you are doing.

Hit Apply. Rastering images takes time. You should check for error messages in the window that opens now, not that there necessarily will be any...

## Installing
#### Inkscape Plugin
Grab atrclaser.py and atrclaser.inx and throw them in:

Windows: C:\Program Files\Inkscape\share\extensions\

Linux: ~/.config/inkscape/extensions/

(Re)start Inkscape.

##### Upgrading Marlin
Remember to make a backup of your current working Marlin if you have one - it's helpful to clearly label is as such. Then download, modify settings, and upload to Arduino:
https://github.com/mayhem2408/buildlog-lasercutter-marlin


##### Help I've never done this before
Please see [the instructions here](https://github.com/TurnkeyTyranny/buildlog-lasercutter-marlin) on how to install the Arduino and RAMPS 1.4 in your cheapo Chinese laser cutter and how to install Marlin.
