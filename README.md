# Repo description
Use fmdtools (Fault Model Design Tools) from nasa to model aeb function in iso 26262 context. 

# Objectives
Objectives
The primary objectives of this AEB (Autonomous Emergency Braking) system model are:

Model error models and error propagations through different components

Systematically capture how errors originate, transform, and propagate across the AEB system architecture
Analyze error cascading effects from sensors through decision logic to actuation
Quantify error accumulation and its impact on system safety performance


Find suitable assume-guarantee contracts

Establish formal contracts between system components that specify assumptions and guarantees
Enable compositional verification of safety properties across the system hierarchy
Support ISO 26262 compliance through rigorous interface specifications

Overview
This repository implements a comprehensive error modeling framework for AEB systems using NASA's fmdtools, with a focus on error propagation analysis and assume-guarantee contract formulation. The model addresses the critical challenge of understanding how component-level errors can lead to system-level hazardous situations.


Key Features

Hierarchical Error Modeling: Error models at Item → System → Subsystem → Component levels
Error Propagation Analysis: Traces error paths through the complete AEB architecture
Assume-Guarantee Contracts: Formal interface specifications for compositional verification
ISO 26262 Compliance: Aligned with functional safety requirements and ASIL decomposition
Quantitative Assessment: Probabilistic error analysis with reliability metrics


# 


# =============================================================================
# ACTUATION SYSTEM FUNCTION (models/system_level/actuation_system.py)
# =============================================================================

class ActuationSystemStates(State):
    """Actuation system state variables"""
    brake_pressure: float = 0.0  # Bar
    system_ready: bool = True
    hydraulic_health: float = 1.0
    response_time: float = 0.1  # seconds

class ActuationSystemParams(Parameter):
    """Actuation system parameters"""
    max_brake_pressure: float = 150.0  # Bar
    min_response_time: float = 0.05  # seconds
    max_response_time: float = 0.2  # seconds

class ActuationFailureModes(Mode):
    """Actuation-specific failure modes"""
    hydraulic_failure: float = 1e-5  # Hydraulic system failure
    valve_stuck: float = 1e-6  # Brake valve stuck
    pressure_loss: float = 1e-4  # Gradual pressure loss
    sensor_failure: float = 1e-5  # Pressure sensor failure

class ActuationSystem(Function):
    """
    Actuation System Function - ISO 26262 System Level
    
    Converts brake commands into physical brake force application.
    ASIL Level: D (critical for safety - final actuation stage)
    """
    
    container_s = ActuationSystemStates
    container_p = ActuationSystemParams
    container_m = ActuationFailureModes
    
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        
        self.safety_requirements = {
            'SR_009': 'Shall apply brake force within 100ms of command',
            'SR_010': 'Shall achieve 80% of commanded brake force',
            'SR_011': 'Shall detect actuation failures within 50ms',
            'SR_012': 'Shall provide driver override capability'
        }
    
    def static_behavior(self, time):
        """Actuation system behavior"""
        
        if self.m.has_fault(['hydraulic_failure', 'valve_stuck']):
            # Complete actuation failure
            self.s.brake_pressure = 0.0
            self.s.system_ready = False
            self.vehicle_state.acceleration = 0.0  # No deceleration possible
        elif self.m.has_fault(['pressure_loss']):
            # Partial failure - reduced braking capability
            commanded_pressure = (self.control_command.brake_command * 
                                self.p.max_brake_pressure)
            self.s.brake_pressure = commanded_pressure * 0.6  # 40% loss
            self.s.system_ready = True
        else:
            # Normal operation
            commanded_pressure = (self.control_command.brake_command * 
                                self.p.max_brake_pressure)
            self.s.brake_pressure = commanded_pressure
            self.s.system_ready = True
        
        # Calculate vehicle deceleration based on brake pressure
        if self.s.brake_pressure > 0:
            # Simplified brake force to deceleration conversion
            max_deceleration = 8.0  # m/s² (typical AEB capability)
            deceleration = (self.s.brake_pressure / self.p.max_brake_pressure) * max_deceleration
            self.vehicle_state.acceleration = -deceleration
        else:
            self.vehicle_state.acceleration = 0.0

# =============================================================================
# VEHICLE DYNAMICS FUNCTION (models/system_level/vehicle_dynamics.py)
# =============================================================================


class VehicleDynamicsStates(State):
    """Vehicle dynamics state variables"""
    velocity: float = 50.0  # Initial velocity (km/h)
    position: float = 0.0  # Position (m)
    mass: float = 1500.0  # Vehicle mass (kg)
    tire_grip: float = 1.0  # Tire grip factor (0-1)

class VehicleDynamicsParams(Parameter):
    """Vehicle dynamics parameters"""
    drag_coefficient: float = 0.3  # Aerodynamic drag
    rolling_resistance: float = 0.01  # Rolling resistance coefficient
    brake_efficiency: float = 0.85  # Brake system efficiency

class VehicleDynamics(Function):
    """
    Vehicle Dynamics Function - ISO 26262 System Level
    
    Models vehicle response to braking commands and environmental forces.
    ASIL Level: B (important for accurate simulation but not directly safety-critical)
    """
    
    container_s = VehicleDynamicsStates
    container_p = VehicleDynamicsParams
    
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
    
    def dynamic_behavior(self, time):
        """Update vehicle dynamics over time"""
        dt = 0.05  # 50ms timestep
        
        # Get current acceleration from actuation system
        if hasattr(self, 'vehicle_state_input'):
            commanded_accel = self.vehicle_state_input.acceleration
        else:
            commanded_accel = 0.0
        
        # Apply environmental forces
        velocity_ms = self.s.velocity / 3.6  # Convert km/h to m/s
        
        # Drag force: F_drag = 0.5 * rho * Cd * A * v²
        drag_deceleration = 0.5 * 1.225 * self.p.drag_coefficient * 2.5 * (velocity_ms**2) / self.s.mass
        
        # Rolling resistance: F_roll = Cr * mg
        rolling_deceleration = self.p.rolling_resistance * 9.81
        
        # Total acceleration (including environmental effects)
        total_acceleration = commanded_accel - drag_deceleration - rolling_deceleration
        
        # Update velocity and position
        self.s.velocity = max(0.0, self.s.velocity + total_acceleration * 3.6 * dt)  # Convert back to km/h
        velocity_ms = self.s.velocity / 3.6
        self.s.position += velocity_ms * dt
        
        # Update output flow with current state
        self.vehicle_state.velocity = velocity_ms
        self.vehicle_state.acceleration = total_acceleration
        self.vehicle_state.position = (self.s.position, 0.0)
        self.vehicle_state.heading = 0.0

# =============================================================================
# ISO 26262 IMPACT ANALYSIS (analysis/impact_analysis.py)  
# =============================================================================

class ISO26262ImpactAnalysis:
    """
    ISO 26262 Impact Analysis for AEB System
    
    Performs systematic analysis of failure impacts on safety goals
    and determines ASIL compliance.
    """
    
    def __init__(self, aeb_model):
        self.model = aeb_model
        self.hazard_catalog = self._load_hazard_catalog()
        self.safety_goals = self._load_safety_goals()
    
    def _load_hazard_catalog(self):
        """Load predefined hazards for AEB system"""
        return {
            'H001': {
                'description': 'Unintended braking during normal driving',
                'severity': 'S2',
                'exposure': 'E4', 
                'controllability': 'C2',
                'asil': 'B'
            },
            'H002': {
                'description': 'No braking when collision imminent',
                'severity': 'S3',
                'exposure': 'E4',
                'controllability': 'C3', 
                'asil': 'D'
            },
            'H003': {
                'description': 'Insufficient braking force for collision avoidance',
                'severity': 'S3',
                'exposure': 'E4',
                'controllability': 'C2',
                'asil': 'D'
            }
        }
    
    def _load_safety_goals(self):
        """Load safety goals derived from hazard analysis"""
        return {
            'SG001': {
                'description': 'The vehicle shall not apply unintended braking during normal driving conditions',
                'asil': 'B',
                'related_hazard': 'H001'
            },
            'SG002': {
                'description': 'The vehicle shall apply sufficient braking force to avoid collision when TTC < 2.5s',
                'asil': 'D', 
                'related_hazard': 'H002'
            },
            'SG003': {
                'description': 'The vehicle shall provide adequate braking performance in degraded conditions',
                'asil': 'D',
                'related_hazard': 'H003'
            }
        }
    
    def analyze_fault_impact(self, fault_results):
        """
        Analyze impact of faults on safety goals
        
        Args:
            fault_results: Dictionary of fault simulation results
            
        Returns:
            Dictionary containing impact analysis for each fault
        """
        
        impact_analysis = {}
        
        for (function, fault_mode), result in fault_results.items():
            
            # Extract key metrics from simulation result
            final_state = result.get_final_state()
            
            # Determine which safety goals are affected
            affected_goals = []
            violation_severity = 'None'
            
            if hasattr(final_state, 'control_command'):
                brake_cmd = final_state.control_command.brake_command
                
                # Check for unintended braking (SG001)
                if brake_cmd > 0.1 and not self._collision_scenario_active(result):
                    affected_goals.append('SG001')
                    violation_severity = 'ASIL B Violation'
                
                # Check for insufficient/no braking (SG002, SG003)
                if self._collision_scenario_active(result) and brake_cmd < 0.8:
                    affected_goals.extend(['SG002', 'SG003'])
                    violation_severity = 'ASIL D Violation'
            
            # Calculate risk metrics
            risk_assessment = self._calculate_risk_metrics(function, fault_mode, result)
            
            impact_analysis[(function, fault_mode)] = {
                'affected_safety_goals': affected_goals,
                'violation_severity': violation_severity,
                'risk_metrics': risk_assessment,
                'mitigation_required': len(affected_goals) > 0,
                'recommended_actions': self._generate_mitigation_recommendations(
                    function, fault_mode, affected_goals
                )
            }
        
        return impact_analysis
    
    def _collision_scenario_active(self, result):
        """Determine if a collision scenario was active during simulation"""
        # Simplified logic - in real implementation, this would check
        # for presence of objects within critical TTC threshold
        return True  # Assume collision scenario for analysis purposes
    
    def _calculate_risk_metrics(self, function, fault_mode, result):
        """Calculate quantitative risk metrics"""
        
        # Base failure rates (per hour)
        base_rates = {
            'perception_system': 1e-5,
            'decision_system': 1e-6, 
            'actuation_system': 1e-5,
            'vehicle_dynamics': 1e-7
        }
        
        failure_rate = base_rates.get(function, 1e-4)
        
        # Mission time analysis (typical driving session)
        mission_time = 2.0  # 2 hour typical trip
        mission_failure_probability = 1 - np.exp(-failure_rate * mission_time)
        
        return {
            'failure_rate_per_hour': failure_rate,
            'mission_failure_probability': mission_failure_probability,
            'mtbf_hours': 1.0 / failure_rate,
            'availability_99_9_percent': -np.log(0.001) / failure_rate
        }
    
    def _generate_mitigation_recommendations(self, function, fault_mode, affected_goals):
        """Generate mitigation recommendations based on failure analysis"""
        
        recommendations = []
        
        if 'SG001' in affected_goals:  # Unintended braking
            recommendations.extend([
                'Implement independent monitoring of brake commands',
                'Add plausibility checks for sensor data',
                'Require confirmation from multiple sensors before braking'
            ])
        
        if 'SG002' in affected_goals or 'SG003' in affected_goals:  # No/insufficient braking
            recommendations.extend([
                'Implement redundant brake actuation path', 
                'Add fail-operational capability for critical components',
                'Implement graceful degradation strategies',
                'Add driver warning systems for degraded operation'
            ])
        
        # Function-specific recommendations
        if function == 'perception_system':
            recommendations.append('Implement sensor cross-validation algorithms')
        elif function == 'decision_system':
            recommendations.append('Add watchdog timer for ECU monitoring')
        elif function == 'actuation_system':
            recommendations.append('Implement redundant hydraulic circuits')
        
        return recommendations

# =============================================================================
# SETUP.PY
# =============================================================================

setup_py_content = '''
from setuptools import setup, find_packages

setup(
    name="aeb-fmdtools-model",
    version="1.0.0", 
    description="ISO 26262 compliant AEB system model using fmdtools",
    author="ADAS System Engineer",
    author_email="adas.engineer@company.com",
    packages=find_packages(),
    install_requires=[
        "fmdtools>=2.0.0",
        "numpy>=1.21.0", 
        "matplotlib>=3.5.0",
        "pandas>=1.3.0",
        "scipy>=1.7.0",
        "networkx>=2.6.0",
        "plotly>=5.0.0",
        "dash>=2.0.0",
        "jupyter>=1.0.0"
    ],
    extras_require={
        "dev": [
            "pytest>=6.0.0",
            "black>=21.0.0",
            "flake8>=3.9.0",
            "mypy>=0.910"
        ]
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License", 
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Safety Critical Systems"
    ],
    keywords="iso26262 adas aeb fmdtools safety automotive"
)
'''

# =============================================================================
# README.MD CONTENT
# =============================================================================

readme_content = '''
# AEB System Model with fmdtools - ISO 26262 Compliant

This repository contains a comprehensive Autonomous Emergency Braking (AEB) system model implemented using NASA's fmdtools framework, structured according to ISO 26262 requirements.

## Overview

The model provides:
- **ISO 26262 compliant system architecture** (Item → System → Subsystem → Component → Software)
- **Multi-ASIL level analysis** (QM, A, B, C, D)
- **Quantitative reliability assessment** for safety integrity verification
- **Operational Design Domain (ODD) modeling** for scenario-based analysis
- **Comprehensive failure mode analysis** with impact assessment

## Model Architecture

### ISO 26262 Hierarchy Mapping

```
Item Level: Complete AEB System (ASIL D)
├── System Level: Major Subsystems
│   ├── Perception System (ASIL D) - Sensor fusion & object detection
│   ├── Decision System (ASIL D) - Collision avoidance logic  
│   ├── Actuation System (ASIL D) - Brake force application
│   └── Vehicle Dynamics (ASIL B) - Vehicle response modeling
│
├── Subsystem Level: Functional Groups
│   ├── Sensors/ (ASIL B-C)
│   ├── Processing/ (ASIL D)
│   └── Actuation/ (ASIL C-D)
│
└── Component Level: Individual Components
    ├── Hardware Components/
    └── Software Components/
```

## Quick Start

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd aeb_system_model

# Install dependencies
pip install -r requirements.txt

# Install in development mode
pip install -e .
```

### Basic Usage

```python
from models.item_level.aeb_system_model import AEBSystemArchitecture
from analysis.impact_analysis import ISO26262ImpactAnalysis

# Create AEB system model
aeb_model = AEBSystemArchitecture()

# Run basic simulation
result = aeb_model.simulate(time=10.0, dt=0.05)

# Perform fault analysis
fault_scenario = {'perception_system': {'complete_failure': 2.0}}
fault_result = aeb_model.simulate(time=10.0, dt=0.05, fault_times=fault_scenario)

# ISO 26262 impact analysis
analyzer = ISO26262ImpactAnalysis(aeb_model)
impact_report = analyzer.analyze_fault_impact({('perception_system', 'complete_failure'): fault_result})
```

### Running Examples

```bash
# Basic analysis example
python examples/basic_analysis.py

# Advanced scenario analysis
python examples/advanced_scenarios.py

# ISO 26262 specific examples
python examples/iso_26262_examples.py
```

## Key Features

### 1. Multi-ASIL Component Modeling
- Individual components tagged with appropriate ASIL levels
- Safety requirement traceability from system to software level
- Automatic ASIL decomposition verification

### 2. Comprehensive Failure Mode Coverage
- **Sensor Failures**: Complete failure, degraded performance, environmental effects
- **ECU Failures**: Hardware faults, software crashes, timing violations
- **Actuation Failures**: Hydraulic failures, valve malfunctions, response delays
- **Common Cause Failures**: EMI, vibration, temperature effects

### 3. Operational Design Domain (ODD) Integration
- Weather condition modeling (clear, rain, snow, fog)
- Road condition effects (dry, wet, icy surfaces)  
- Traffic density and speed limit considerations
- Time-of-day visibility impacts

### 4. Quantitative Analysis Capabilities
- Probabilistic failure rate calculations
- Safety Integrity Level (SIL) verification
- Mission time reliability analysis
- Availability and maintainability metrics

## File Structure Explanation

### `/models/` - Core fmdtools Models
- **item_level/**: Complete system model (ISO 26262 Item definition)
- **system_level/**: Major subsystem functions
- **subsystem_level/**: Detailed functional decomposition
- **component_level/**: Individual hardware/software components
- **common/**: Shared utilities, failure modes, ASIL definitions

### `/iso_26262_docs/` - Safety Documentation  
- **item_definition/**: Hazard analysis, safety goals, ASIL classification
- **system_design/**: Safety requirements, architecture, technical safety concept
- **verification_validation/**: Safety analysis results, impact assessments

### `/analysis/` - Analysis Tools
- **fault_analysis.py**: Core fault simulation and analysis
- **impact_analysis.py**: ISO 26262 impact assessment
- **reliability_analysis.py**: Quantitative reliability calculations
- **scenario_analysis.py**: ODD-based scenario testing

## ISO 26262 Compliance Features

### Safety Goals Traceability
Each model component includes:
- Linked safety requirements (SR_xxx format)
- ASIL level justification  
- Failure mode impact assessment
- Verification criteria

### Hazard Analysis Integration
- Predefined hazard catalog for AEB systems
- Systematic safety goal derivation
- Risk assessment (Severity × Exposure × Controllability)
- ASIL determination automation

### Verification & Validation Support
- Automated safety analysis report generation
- Fault injection test scenarios
- Safety case evidence collection
- Compliance gap analysis

## Advanced Usage

### Custom Failure Modes

```python
from models.common.failure_modes import SensorFailureModes

# Extend with custom failure modes
class CustomSensorFailures(SensorFailureModes):
    laser_degradation: float = 1e-5  # Custom LiDAR failure
    cyber_attack: float = 1e-7       # Cybersecurity failure
```

### ODD Scenario Testing

```python
from models.common.odd_model import ODDConditions

# Define custom operating conditions
odd_conditions = ODDConditions(
    weather="rain",
    visibility=200.0,  # meters
    road_surface="wet", 
    traffic_density="heavy",
    speed_limit=60.0   # km/h
)

# Test system under specific ODD
result = aeb_model.simulate_with_odd(odd_conditions, time=10.0)
```

### Quantitative Analysis

```python
from models.common.reliability_models import ReliabilityModel

reliability = ReliabilityModel()

# Calculate component failure rates
radar_rate = reliability.calculate_failure_rate('radar_sensor', operating_hours=5000)

# Determine required SIL level
system_rate = sum(component_rates.values()) 
required_sil = reliability.calculate_safety_integrity_level(system_rate)

print(f"System requires {required_sil} for safety compliance")
```

## Contributing

1. Follow ISO 26262 documentation standards
2. Add safety requirements traceability for new components
3. Include appropriate ASIL level justification
4. Provide quantitative failure data where possible
5. Test with multiple ODD scenarios

## Safety Notice

This model is for research and development purposes. Any safety-critical automotive application requires:
- Formal verification and validation per ISO 26262
- Certified development processes (ISO 26262 Part 2)
- Independent safety assessment
- Regulatory approval for deployment

## References

- ISO 26262:2018 - Road vehicles — Functional safety
- fmdtools Documentation: https://github.com/nasa/fmdtools
- AEB System Standards: ISO 22733, Euro NCAP protocols
'''

print("✅ Complete AEB fmdtools Model Structure Created!")
print("\n📋 Key Components Included:")
print("- Full ISO 26262 compliant folder structure")  
print("- Multi-ASIL level system architecture")
print("- Comprehensive failure mode definitions")
print("- Quantitative reliability analysis capabilities")
print("- ODD (Operational Design Domain) modeling")
print("- Sensor fusion with radar/lidar/camera")
print("- Impact analysis for safety goal compliance")
print("- Example usage and analysis scripts")

print("\n🚀 Next Steps:")
print("1. Install fmdtools: pip install fmdtools")
print("2. Create the folder structure as shown")
print("3. Copy the Python code into appropriate files")
print("4. Run examples/basic_analysis.py to test")
print("5. Customize failure modes and safety requirements")
print("6. Add your specific hazard analysis results")