PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//11394118/1553794/2.50/2/2/Zener Diode

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r105_85"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.85) (shapeHeight 1.05))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "SOD3716X135N" (originalName "SOD3716X135N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r105_85) (pt -1.8, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef r105_85) (pt 1.8, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.575 1.655) (pt 2.575 1.655) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.575 1.655) (pt 2.575 -1.655) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.575 -1.655) (pt -2.575 -1.655) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.575 -1.655) (pt -2.575 1.655) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 0.8) (pt 1.34 0.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.34 0.8) (pt 1.34 -0.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.34 -0.8) (pt -1.34 -0.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 -0.8) (pt -1.34 0.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 0.275) (pt -0.815 0.8) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.325 0.8) (pt 1.34 0.8) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.34 -0.8) (pt 1.34 -0.8) (width 0.2))
		)
	)
	(symbolDef "MMSZ5231B-TP" (originalName "MMSZ5231B-TP")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 140 mils -15 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 600 mils 0 mils) (rotation 180) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 460 mils -15 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 80 mils) (pt 200 mils -80 mils) (width 6 mils))
		(line (pt 200 mils 80 mils) (pt 240 mils 100 mils) (width 6 mils))
		(line (pt 160 mils -100 mils) (pt 200 mils -80 mils) (width 6 mils))
		(line (pt 100 mils 0 mils) (pt 200 mils 0 mils) (width 6 mils))
		(line (pt 500 mils 0 mils) (pt 400 mils 0 mils) (width 6 mils))
		(poly (pt 200 mils 0 mils) (pt 400 mils 100 mils) (pt 400 mils -100 mils) (pt 200 mils 0 mils) (width 10  mils))
		(attr "RefDes" "RefDes" (pt 400 mils 350 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 400 mils 250 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "MMSZ5231B-TP" (originalName "MMSZ5231B-TP") (compHeader (numPins 2) (numParts 1) (refDesPrefix Z)
		)
		(compPin "1" (pinName "K") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "A") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "MMSZ5231B-TP"))
		(attachedPattern (patternNum 1) (patternName "SOD3716X135N")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Mouser Part Number" "833-MMSZ5231B-TP")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Micro-Commercial-Components-MCC/MMSZ5231B-TP?qs=ZNK0BnemlqEexvzDQxpNYA%3D%3D")
		(attr "Manufacturer_Name" "MCC")
		(attr "Manufacturer_Part_Number" "MMSZ5231B-TP")
		(attr "Description" "Zener Diodes")
		(attr "<Hyperlink>" "https://mccsemi.com/pdf/Products/MMSZ5221-MMSZ5267(SOD-123).pdf")
		(attr "<Component Height>" "1.35")
		(attr "<STEP Filename>" "MMSZ5231B-TP.stp")
		(attr "<STEP Offsets>" "X=-0.28;Y=2.8;Z=0.72")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=0")
	)

)
