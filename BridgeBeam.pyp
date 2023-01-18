<?xml version="1.0" encoding="utf-8"?>
<Element>
    <Script>
        <Name>BridgeBeam.py</Name>
        <Title>CreateBridgeBeam</Title>
        <Version>1.0</Version>
    </Script>
    <Page>
        <Parameter>
            <Name>Expander1</Name>
            <Text>Балка</Text>
            <ValueType>Expander</ValueType>
            <Parameter>
                <Name>Length</Name>
                <Text>Довжина</Text>
                <Value>12000</Value>
                <MinValue>12000</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Height</Name>
                <Text>Висота</Text>
                <Value>1100</Value>
                <MinValue>1100</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Separator2</Name>
                <ValueType>Separator</ValueType>
            </Parameter>
            <Parameter>
                <Name>TopWidth</Name>
                <Text>Ширина верхньої полки</Text>
                <Value>600</Value>
                <MinValue>600</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>TopHeight</Name>
                <Text>Висота верхньої полки</Text>
                <Value>320</Value>
                <MinValue>320</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Separator3</Name>
                <ValueType>Separator</ValueType>
            </Parameter>
            <Parameter>
                <Name>BottomWidth</Name>
                <Text>Ширина нижньої полки</Text>
                <Value>480</Value>
                <MinValue>480</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>BottomHeight</Name>
                <Text>Висота нижньої полки</Text>
                <Value>313</Value>
                <MinValue>313</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Separator4</Name>
                <ValueType>Separator</ValueType>
            </Parameter>
            <Parameter>
                <Name>MiddleWidth</Name>
                <Text>Товщина ребра</Text>
                <Value>160</Value>
                <MinValue>160</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>MiddleHeight</Name>
                <Text>Висота ребра</Text>
                <Value>467</Value>
                <MinValue>467</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Separator5</Name>
                <ValueType>Separator</ValueType>
            </Parameter>
            <Parameter>
                <Name>HoleDepth</Name>
                <Text>Глибина до строповочного отвору</Text>
                <Value>350</Value>
                <MinValue>350</MinValue>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>HoleHeight</Name>
                <Text>Висота до строповочного отвору</Text>
                <Value>540</Value>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>Separator6</Name>
                <ValueType>Separator</ValueType>
            </Parameter>
            <Parameter>
                <Name>AngleX</Name>
                <Text>Поворот відносно осі X</Text>
                <Value>0</Value>
                <ValueType>Angle</ValueType>
            </Parameter>
            <Parameter>
                <Name>AngleY</Name>
                <Text>Поворот відносно осі Y</Text>
                <Value>0</Value>
                <ValueType>Angle</ValueType>
            </Parameter>
            <Parameter>
                <Name>AngleZ</Name>
                <Text>Поворот відносно осі Z</Text>
                <Value>0</Value>
                <ValueType>Angle</ValueType>
            </Parameter>
        </Parameter>
        <Parameter>
            <Name>Expander2</Name>
            <Text>Армування</Text>
            <ValueType>Expander</ValueType>
            <Parameter>
                <Name>ConcreteGrade</Name>
                <Text>Марка бетону</Text>
                <Value>4</Value>
                <ValueType>ReinfConcreteGrade</ValueType>
            </Parameter>
            <Parameter>
                <Name>SteelGrade</Name>
                <Text>Марка сталі</Text>
                <Value>4</Value>
                <ValueType>ReinfSteelGrade</ValueType>
            </Parameter>
            <Parameter>
                <Name>BarDiameter</Name>
                <Text>Діаметр стрижнів</Text>
                <Value>10</Value>
                <ValueType>ReinfBarDiameter</ValueType>
            </Parameter>
            <Parameter>
                <Name>ConcreteCover</Name>
                <Text>Захисний шар</Text>
                <Value>50</Value>
                <ValueType>ReinfConcreteCover</ValueType>
            </Parameter>
            <Parameter>
                <Name>BarSpacing</Name>
                <Text>Крок армування</Text>
                <Value>100</Value>
                <ValueType>Length</ValueType>
                <MinValue>2 * BarDiameter</MinValue>
            </Parameter>
            <Parameter>
                <Name>BendingRoller</Name>
                <Text>Bending roller</Text>
                <Value>4</Value>
                <ValueType>ReinfBendingRoller</ValueType>
            </Parameter>
            <Parameter>
                <Name>BarHeight</Name>
                <Text>Висота над балкою</Text>
                <Value>145</Value>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>HookLength</Name>
                <Text>Довжина загнутої частини</Text>
                <Value>110</Value>
                <ValueType>Length</ValueType>
            </Parameter>
            <Parameter>
                <Name>BarDepth</Name>
                <Text>Глибина входження в балку</Text>
                <Value>200</Value>
                <ValueType>Length</ValueType>
            </Parameter>
        </Parameter>
    </Page>    
</Element>
