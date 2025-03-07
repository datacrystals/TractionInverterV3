SamacSys ECAD Model
18521389/1553794/2.50/2/3/Capacitor

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "s300_h200"
		(holeDiam 2)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 3.000) (shapeHeight 3.000))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 3.000) (shapeHeight 3.000))
	)
	(padStyleDef "c300_h200"
		(holeDiam 2)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 3.000) (shapeHeight 3.000))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 3.000) (shapeHeight 3.000))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "ALH82A750CB600" (originalName "ALH82A750CB600")
		(multiLayer
			(pad (padNum 1) (padStyleRef s300_h200) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c300_h200) (pt 10.000, 0.000) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 5.000, 0.000) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -11 16) (pt 21 16) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 21 16) (pt 21 -16) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 21 -16) (pt -11 -16) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -11 -16) (pt -11 16) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -10 0) (pt -10 0) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(arc (pt 5, 0) (radius 15) (startAngle 180) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 20 0) (pt 20 0) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(arc (pt 5, 0) (radius 15) (startAngle .0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -10 0) (pt -10 0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 5, 0) (radius 15) (startAngle 180) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 20 0) (pt 20 0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 5, 0) (radius 15) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -10.6 0) (pt -10.6 0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -10.55, 0) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -10.5 0) (pt -10.5 0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -10.55, 0) (radius 0.05) (startAngle .0) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "ALH82A750CB600" (originalName "ALH82A750CB600")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 0 mils -35 mils) (rotation 0]) (justify "UpperLeft") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 500 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 500 mils -35 mils) (rotation 0]) (justify "UpperRight") (textStyleRef "Default"))
		))
		(line (pt 220 mils 100 mils) (pt 220 mils -100 mils) (width 6 mils))
		(line (pt 280 mils 100 mils) (pt 280 mils -100 mils) (width 6 mils))
		(line (pt 200 mils 0 mils) (pt 220 mils 0 mils) (width 6 mils))
		(line (pt 280 mils 0 mils) (pt 300 mils 0 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 350 mils 250 mils) (justify 24) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "ALH82A750CB600" (originalName "ALH82A750CB600") (compHeader (numPins 2) (numParts 1) (refDesPrefix C)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "ALH82A750CB600"))
		(attachedPattern (patternNum 1) (patternName "ALH82A750CB600")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Mouser Part Number" "80-ALH82A750CB600")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/KEMET/ALH82A750CB600?qs=IPgv5n7u5QYD2qLIrnuv%2FQ%3D%3D")
		(attr "Manufacturer_Name" "KEMET")
		(attr "Manufacturer_Part_Number" "ALH82A750CB600")
		(attr "Description" "75 F 600 V Aluminum Electrolytic Capacitors Radial, Can - Snap-In 1.861Ohm @ 100Hz 3000 Hrs @ 105C")
		(attr "Datasheet Link" "https://search.kemet.com/component-documentation/download/specsheet/ALH82A750CB600")
		(attr "Height" "32 mm")
	)

)
