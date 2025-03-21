SamacSys ECAD Model
15644057/1553794/2.50/3/4/MOSFET N-Channel

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r228.6_139.7"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.397) (shapeHeight 2.286))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "r618_569"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 5.690) (shapeHeight 6.180))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "SIHFR120GE3" (originalName "SIHFR120GE3")
		(multiLayer
			(pad (padNum 1) (padStyleRef r228.6_139.7) (pt -2.286, -3.345) (rotation 0))
			(pad (padNum 2) (padStyleRef r228.6_139.7) (pt 2.286, -3.345) (rotation 0))
			(pad (padNum 3) (padStyleRef r618_569) (pt 0.000, 3.054) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, 0.828) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.175 5.05) (pt 3.175 5.05) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.175 5.05) (pt 3.175 -1) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.175 -1) (pt -3.175 -1) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.175 -1) (pt -3.175 5.05) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -4.175 7.144) (pt 4.175 7.144) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 4.175 7.144) (pt 4.175 -5.488) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 4.175 -5.488) (pt -4.175 -5.488) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -4.175 -5.488) (pt -4.175 7.144) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.175 -1) (pt 3.175 -1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.3 -4.9) (pt -2.3 -4.9) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.3, -4.95) (radius 0.05) (startAngle 90.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.3 -5) (pt -2.3 -5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.3, -4.95) (radius 0.05) (startAngle 270) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "SIHFR120-GE3" (originalName "SIHFR120-GE3")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 0 mils -45 mils) (rotation 0]) (justify "UpperLeft") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 300 mils 400 mils) (rotation 270) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 305 mils 400 mils) (rotation 90]) (justify "UpperLeft") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 300 mils -200 mils) (rotation 90) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 305 mils -200 mils) (rotation 90]) (justify "LowerLeft") (textStyleRef "Default"))
		))
		(line (pt 300 mils 100 mils) (pt 300 mils -100 mils) (width 6 mils))
		(line (pt 300 mils 200 mils) (pt 300 mils 300 mils) (width 6 mils))
		(line (pt 100 mils 0 mils) (pt 200 mils 0 mils) (width 6 mils))
		(line (pt 200 mils 0 mils) (pt 200 mils 200 mils) (width 6 mils))
		(line (pt 300 mils 100 mils) (pt 230 mils 100 mils) (width 6 mils))
		(line (pt 300 mils 200 mils) (pt 230 mils 200 mils) (width 6 mils))
		(line (pt 230 mils 0 mils) (pt 300 mils 0 mils) (width 6 mils))
		(line (pt 230 mils 220 mils) (pt 230 mils 180 mils) (width 6 mils))
		(line (pt 230 mils -20 mils) (pt 230 mils 20 mils) (width 6 mils))
		(line (pt 230 mils 80 mils) (pt 230 mils 120 mils) (width 6 mils))
		(arc (pt 250 mils 100 mils) (radius 150 mils) (startAngle 0) (sweepAngle 360) (width 10  mils))
		(poly (pt 230 mils 100 mils) (pt 270 mils 120 mils) (pt 270 mils 80 mils) (pt 230 mils 100 mils) (width 10  mils))
		(attr "RefDes" "RefDes" (pt 450 mils 150 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "SIHFR120-GE3" (originalName "SIHFR120-GE3") (compHeader (numPins 3) (numParts 1) (refDesPrefix Q)
		)
		(compPin "1" (pinName "G") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "S") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "D") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "SIHFR120-GE3"))
		(attachedPattern (patternNum 1) (patternName "SIHFR120GE3")
			(numPads 3)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
			)
		)
		(attr "Mouser Part Number" "78-SIHFR120-GE3")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Vishay-Siliconix/SIHFR120-GE3?qs=FfLXECYddQTykmVhSg2%2FTQ%3D%3D")
		(attr "Manufacturer_Name" "Vishay")
		(attr "Manufacturer_Part_Number" "SIHFR120-GE3")
		(attr "Description" "MOSFET 100V N-CH")
		(attr "Datasheet Link" "https://www.mouser.in/datasheet/2/427/91266-1768752.pdf")
		(attr "Height" "2.38 mm")
	)

)
