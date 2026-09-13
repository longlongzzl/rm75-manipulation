"""Bounded raw native contact evidence, never an absence-of-collision proof."""
import copy
import math
import numpy as np
from .scene import SceneInvalid


class ToolContactEvidence:
    def __init__(self,tool_entities,object_entities,target_id, *, maximum_pairs=512):
        self.tools=dict(tool_entities);self.objects=dict(object_entities);self.target_id=target_id
        self.maximum_pairs=maximum_pairs;self.rows={};self.steps=0;self.last_time=None

    def observe(self,contacts, *, time_s,stage):
        if not math.isfinite(time_s) or (self.last_time is not None and time_s<=self.last_time):
            raise SceneInvalid('Native contact clock did not advance')
        self.last_time=time_s;self.steps+=1
        for contact in contacts:
            entities=[body.entity for body in contact.bodies]
            if len(entities)!=2:raise SceneInvalid('Native contact body identity invalid')
            matches=[i for i,entity in enumerate(entities) if entity in self.tools]
            if not matches:continue
            index=matches[0];tool=self.tools[entities[index]];other=entities[1-index]
            if other in self.tools:kind='tool_self';name=self.tools[other]
            elif other in self.objects:
                name=self.objects[other];kind='target' if name==self.target_id else 'environment'
            else:raise SceneInvalid('Native contact has an unregistered body')
            points=list(contact.points)
            if not points:continue
            impulses=np.asarray([point.impulse for point in points],float)
            separations=np.asarray([point.separation for point in points],float)
            if impulses.shape!=(len(points),3) or not np.isfinite(impulses).all() or not np.isfinite(separations).all():
                raise SceneInvalid('Invalid native contact point readback')
            impulse=float(np.linalg.norm(impulses.sum(axis=0)));separation=float(separations.min())
            key=(stage,tool,kind,name)
            if key not in self.rows:
                if len(self.rows)>=self.maximum_pairs:raise SceneInvalid('Native contact evidence budget exceeded')
                self.rows[key]=dict(stage=stage,tool_link=tool,kind=kind,other=name,
                    first_time_s=time_s,last_time_s=time_s,contacts=0,positive_impulse_contacts=0,
                    maximum_impulse_Ns=0.,minimum_separation_m=separation)
            row=self.rows[key];row['last_time_s']=time_s;row['contacts']+=1
            row['positive_impulse_contacts']+=int(impulse>0)
            row['maximum_impulse_Ns']=max(row['maximum_impulse_Ns'],impulse)
            row['minimum_separation_m']=min(row['minimum_separation_m'],separation)

    def report(self):
        return dict(source='native_PhysX_contact_points',physics_steps=self.steps,
            pairs=copy.deepcopy(list(self.rows.values())),path_safety_qualified=False,
            absence_of_contact_proves_clearance=False,
            limitation='Kinematic/static and kinematic/self contact callbacks are not complete collision audits; original full-path auditor required')
