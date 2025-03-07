SamacSys ECAD Model
17159803/1553794/2.50/2/3/Resistor

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r330_120"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.2) (shapeHeight 3.3))
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
	(patternDef "RESC6431X65N" (originalName "RESC6431X65N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r330_120) (pt -3, 0) (rotation 0))
			(pad (padNum 2) (padStyleRef r330_120) (pt 3, 0) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.85 1.9) (pt 3.85 1.9) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.85 1.9) (pt 3.85 -1.9) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.85 -1.9) (pt -3.85 -1.9) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.85 -1.9) (pt -3.85 1.9) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.175 1.55) (pt 3.175 1.55) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.175 1.55) (pt 3.175 -1.55) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.175 -1.55) (pt -3.175 -1.55) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.175 -1.55) (pt -3.175 1.55) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0 1.45) (pt 0 -1.45) (width 0.2))
		)
	)
	(symbolDef "RC2512JK-071ML" (originalName "RC2512JK-071ML")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 0 mils -35 mils) (rotation 0]) (justify "UpperLeft") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 700 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 700 mils -35 mils) (rotation 0]) (justify "UpperRight") (textStyleRef "Default"))
		))
		(line (pt 200 mils 50 mils) (pt 500 mils 50 mils) (width 6 mils))
		(line (pt 500 mils 50 mils) (pt 500 mils -50 mils) (width 6 mils))
		(line (pt 500 mils -50 mils) (pt 200 mils -50 mils) (width 6 mils))
		(line (pt 200 mils -50 mils) (pt 200 mils 50 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 550 mils 250 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "RC2512JK-071ML" (originalName "RC2512JK-071ML") (compHeader (numPins 2) (numParts 1) (refDesPrefix R)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "RC2512JK-071ML"))
		(attachedPattern (patternNum 1) (patternName "RESC6431X65N")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Mouser Part Number" "603-RC2512JK-071ML")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/YAGEO/RC2512JK-071ML?qs=Fz%2FrpjPuTcGzVNzRy4mnPw%3D%3D")
		(attr "Manufacturer_Name" "YAGEO")
		(attr "Manufacturer_Part_Number" "RC2512JK-071ML")
		(attr "Description" "1 MOhms +/-5% 1W Chip Resistor 2512 (6432 Metric) Moisture Resistant Thick Film")
		(attr "Datasheet Link" "https://www.yageo.com/upload/media/product/products/datasheet/rchip/PYu-RC_Group_51_RoHS_L_12.pdf")
		(attr "Height" "0.65 mm")
	)

)
