PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//171060/1553794/2.50/3/2/Transistor BJT NPN

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c201_h134"
		(holeDiam 1.34)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.01) (shapeHeight 2.01))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.01) (shapeHeight 2.01))
	)
	(padStyleDef "s201_h134"
		(holeDiam 1.34)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 2.01) (shapeHeight 2.01))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 2.01) (shapeHeight 2.01))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "TO254P483X1010X2035-3P" (originalName "TO254P483X1010X2035-3P")
		(multiLayer
			(pad (padNum 1) (padStyleRef s201_h134) (pt 0, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef c201_h134) (pt 2.54, 0) (rotation 90))
			(pad (padNum 3) (padStyleRef c201_h134) (pt 5.08, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.975 3.345) (pt 8.055 3.345) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 8.055 3.345) (pt 8.055 -1.985) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 8.055 -1.985) (pt -2.975 -1.985) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.975 -1.985) (pt -2.975 3.345) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.725 3.095) (pt 7.805 3.095) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.805 3.095) (pt 7.805 -1.735) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.805 -1.735) (pt -2.725 -1.735) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.725 -1.735) (pt -2.725 3.095) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.725 1.825) (pt -1.455 3.095) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.805 -1.735) (pt 7.805 3.095) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.805 3.095) (pt -2.725 3.095) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.725 3.095) (pt -2.725 0) (width 0.2))
		)
	)
	(symbolDef "MJE3055TG" (originalName "MJE3055TG")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 0 mils -45 mils) (rotation 0]) (justify "UpperLeft") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 400 mils -300 mils) (rotation 90) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 405 mils -300 mils) (rotation 90]) (justify "LowerLeft") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 400 mils 300 mils) (rotation 270) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 405 mils 300 mils) (rotation 90]) (justify "UpperLeft") (textStyleRef "Normal"))
		))
		(line (pt 300 mils 100 mils) (pt 300 mils -100 mils) (width 20 mils))
		(line (pt 300 mils 50 mils) (pt 400 mils 150 mils) (width 6 mils))
		(line (pt 300 mils -50 mils) (pt 400 mils -150 mils) (width 6 mils))
		(line (pt 400 mils -150 mils) (pt 400 mils -200 mils) (width 6 mils))
		(line (pt 400 mils 150 mils) (pt 400 mils 200 mils) (width 6 mils))
		(line (pt 100 mils 0 mils) (pt 300 mils 0 mils) (width 6 mils))
		(arc (pt 350 mils 0 mils) (radius 158.114 mils) (startAngle 0) (sweepAngle 360) (width 10  mils))
		(poly (pt 330 mils -100 mils) (pt 350 mils -80 mils) (pt 370 mils -120 mils) (pt 330 mils -100 mils) (width 10  mils))
		(attr "RefDes" "RefDes" (pt 550 mils 50 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 550 mils -50 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "MJE3055TG" (originalName "MJE3055TG") (compHeader (numPins 3) (numParts 1) (refDesPrefix Q)
		)
		(compPin "1" (pinName "B") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "C") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "E") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "MJE3055TG"))
		(attachedPattern (patternNum 1) (patternName "TO254P483X1010X2035-3P")
			(numPads 3)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
			)
		)
		(attr "Mouser Part Number" "863-MJE3055TG")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/onsemi/MJE3055TG?qs=HVbQlW5zcXUpZ6jjVRRo8A%3D%3D")
		(attr "Manufacturer_Name" "onsemi")
		(attr "Manufacturer_Part_Number" "MJE3055TG")
		(attr "Description" "DC Current Gain Specified to 10 Amperes; High Current Gain - Bandwidth Product - fT = 2.0 MHz (Min) @ IC fT = 500 mAdc; Pb-Free Packages are Available")
		(attr "<Hyperlink>" "https://www.onsemi.com/pub/Collateral/MJE2955T-D.PDF")
		(attr "<Component Height>" "4.83")
		(attr "<STEP Filename>" "MJE3055TG.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
