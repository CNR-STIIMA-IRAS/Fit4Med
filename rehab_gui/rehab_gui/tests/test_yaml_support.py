import ast
import copy
from enum import Enum
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QSpinBox, QDoubleSpinBox, QLCDNumber, QRadioButton, QPushButton, QLineEdit, QProgressBar
from YamlSupport import *
app=QApplication.instance() or QApplication([])

def movement():
    return {'a_movement_definition': {'type':[1], 'side':[1], 'vel_profile':[2],
        'max_velocity':[10.0], 'total_time':[2.0], 'begin_config':[[0,0,0]],
        'end_config':[[.1,0,0]], 'begin_joint_config':[[0,0,0]]},
        'cart_trj3':{'cart_positions':[[0,0,0],[.1,0,0]],'time_from_start':[[0],[2]],'joint_names':['x','y','z']}}

def protocol(duration=True):
    d={'Phases':{'PhaseIsEnabled':[[1]*20],'Modalities':[[0]*20], 'Percentage':[[50]*20]}}
    if duration: d['Phases']['Duration']=[[30]*20]
    return d

def extracted(filename, name):
    source=Path(__file__).resolve().parents[1]/filename
    node=next(n for c in ast.parse(source.read_text()).body if isinstance(c,ast.ClassDef)
              for n in c.body if isinstance(n,ast.FunctionDef) and n.name==name)
    node.decorator_list=[]
    ns=dict(globals(),deepcopy=copy.deepcopy,ExerciseType=Enum('ExerciseType', {'NONE':0,'REACHING':1,'HAND_TO_MOUTH':2}))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(source),'exec'),ns)
    return ns[name]

class YamlTests(unittest.TestCase):
    def test_movement_validation(self):
        validate_movement(movement(),execution=True)
        for mutate in [lambda d:d.update(a_movement_definition=None),
                       lambda d:d['cart_trj3']['cart_positions'][0].append(0),
                       lambda d:d['cart_trj3']['time_from_start'].pop(),
                       lambda d:d['cart_trj3']['time_from_start'].__setitem__(1,[0]),
                       lambda d:d['a_movement_definition'].__setitem__('type',[99]),
                       lambda d:d['a_movement_definition'].__setitem__('total_time',[float('nan')]),
                       lambda d:d['a_movement_definition'].__setitem__('total_time',['2']),
                       lambda d:d['a_movement_definition'].__setitem__('begin_config',[[1,0,0]])]:
            data=movement();mutate(data)
            with self.assertRaises(YamlDataError):validate_movement(data,execution=True)

    def test_protocol_validation_and_legacy(self):
        validate_protocol(protocol(False))
        for field in ['PhaseIsEnabled','Modalities','Percentage','Duration']:
            d=protocol();d['Phases'][field][0].pop()
            with self.assertRaises(YamlDataError):validate_protocol(d)
        d=protocol();d['Phases']['Duration'][0][0]=float('inf')
        with self.assertRaises(YamlDataError):validate_protocol(d)

    def test_read_diagnostics_and_reject_duplicates_tags_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'file.yaml'
            for text in ['', 'a: [', 'a: 1\na: 2\n', '!!python/object/apply:os.system [echo bad]']:
                p.write_text(text)
                with self.assertRaises(YamlDataError) as error:read_yaml(p,validate_movement)
                self.assertIn('SHA-256:',str(error.exception))
                self.assertIn(str(p),str(error.exception))

    def test_atomic_roundtrip_numpy_and_no_global_dumper_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'file.yaml';data=movement()
            data['a_movement_definition']['total_time']=[np.float64(2)]
            data['cart_trj3']['cart_positions']=np.array([[0,0,0],[.1,0,0]])
            atomic_save_yaml(p,data,validate_movement)
            self.assertNotIn('!!python',p.read_text())
            self.assertEqual(read_yaml(p,validate_movement),movement())

    def test_failed_save_preserves_original_and_removes_temp(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'file.yaml';p.write_bytes(b'previous')
            with patch('YamlSupport.os.replace',side_effect=OSError('simulated replace failure')):
                with self.assertRaises(OSError):atomic_save_yaml(p,movement(),validate_movement)
            self.assertEqual(p.read_bytes(),b'previous')
            self.assertEqual(list(Path(tmp).iterdir()),[p])
            with self.assertRaises(YamlDataError):atomic_save_yaml(p,{},validate_movement)
            self.assertEqual(p.read_bytes(),b'previous')

    def protocol_view(self):
        view=types.SimpleNamespace()
        view.ui_main=types.SimpleNamespace(Vmax=10,PhaseDuration=4)
        view._min_speed_ovr=10;view.ProtocolData={'old':True}
        view.spinBoxSpeedOvr=[QSpinBox() for _ in range(20)]
        view.spinBoxDuration=[QSpinBox() for _ in range(20)]
        for w in view.spinBoxSpeedOvr+view.spinBoxDuration:w.setRange(0,1000);w.setValue(77)
        view.progressBarPhases=[QProgressBar() for _ in range(20)]
        view.lcdNumberPhases=[QLCDNumber() for _ in range(20)]
        view.ui=types.SimpleNamespace(lcdNumber_SinglePhaseDuration=QLCDNumber(),lcdNumber_MaxVel=QLCDNumber(),lcdNumberExerciseTotalTime=QLCDNumber())
        return view

    def test_protocol_commit_blocks_signals_and_legacy_resets_duration(self):
        v=self.protocol_view();changed=[]
        for w in v.spinBoxDuration:w.valueChanged.connect(changed.append)
        extracted('TrainingProtocolWindow.py','_applyProtocol')(v,protocol(False))
        self.assertEqual(changed,[])
        self.assertEqual([w.value() for w in v.spinBoxDuration],[60]*20)
        self.assertEqual(v.TotalTrainingTime,1200)
        self.assertEqual(v.ProtocolData['Phases']['Duration'],[[60]*20])
        v.spinBoxDuration[0].setValue(61);self.assertEqual(changed,[61])

    def test_protocol_invalid_does_not_mutate(self):
        v=self.protocol_view();old=v.ProtocolData;d=protocol();d['Phases']['Duration'][0][3]=2000
        with self.assertRaises(YamlDataError):extracted('TrainingProtocolWindow.py','_applyProtocol')(v,d)
        self.assertIs(v.ProtocolData,old)
        self.assertEqual([w.value() for w in v.spinBoxDuration],[77]*20)

    def test_protocol_widget_exception_rolls_back(self):
        v=self.protocol_view();old=v.ProtocolData
        class FailOnce(QSpinBox):
            def setValue(self,x):
                if x==30 and not getattr(self,'failed',False):
                    self.failed=True;raise RuntimeError('simulated widget failure')
                super().setValue(x)
        w=FailOnce();w.setRange(0,1000);w.setValue(77);v.spinBoxDuration[10]=w
        with self.assertRaises(RuntimeError):extracted('TrainingProtocolWindow.py','_applyProtocol')(v,protocol())
        self.assertIs(v.ProtocolData,old)
        self.assertEqual([w.value() for w in v.spinBoxDuration],[77]*20)
        self.assertFalse(hasattr(v,'TotalTrainingTime'))
        self.assertFalse(w.signalsBlocked())

    def test_movement_commit_and_rollback(self):
        v=types.SimpleNamespace(main_app=types.SimpleNamespace(movement_loaded=True,Vmax=99,PhaseDuration=99),TrjYamlData={'old':True})
        v.ui=types.SimpleNamespace()
        for n in ['radioButton_TypeOfExercise_Reaching','radioButton_TypeOfExercise_HandtoMouth','radioButton_SideLeft','radioButton_SideRight']:
            setattr(v.ui,n,QRadioButton())
        v.ui.lineEdit_MovementName=QLineEdit('old')
        v.ui.doubleSpinBox_MoveTime=QDoubleSpinBox();v.ui.doubleSpinBox_MoveTime.setRange(0,100)
        v.ui.pushButton_SAVEMovement=QPushButton();v.ui.pushButton_CREATEMovement=QPushButton()
        for n in ['lcdNumber_EndPos_X','lcdNumber_EndPos_Y','lcdNumber_EndPos_Z']:setattr(v.ui,n,QLCDNumber())
        apply=extracted('RehabilitationMovementWindow.py','_applyMovement')
        d=movement();apply(v,d,'new')
        self.assertIs(v.TrjYamlData,d);self.assertEqual(v.main_app.Vmax,10)
        bad=movement();bad['a_movement_definition']['end_config']=[[0,0,0]]
        with self.assertRaises(YamlDataError):apply(v,bad,'bad')
        self.assertIs(v.TrjYamlData,d);self.assertEqual(v.ui.lineEdit_MovementName.text(),'new')
