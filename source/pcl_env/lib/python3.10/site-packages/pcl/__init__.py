import os
import imp
import json
import yaml

class PCL:
    
    def __init__(self, file):
        self.file = file
        self.abspath = os.path.abspath(file)
        self.name = os.path.basename(self.abspath).replace(".pcl", "")
        self.dirname = os.path.dirname(self.abspath)
        self.module = None

    def _in(self):
        self.module = imp.load_source(self.name, self.abspath)
    
    def to_dict(self):
        self._in()
        return {k: getattr(self.module, k) for k in dir(self.module) if not k.startswith('_')}
    
    def to_yaml(self):
        return yaml.dump(self._to_dict(), default_flow_style=False)
    
    def to_json(self):
        return json.dumps(self._to_dict())
    
    # TODO: consider supporting an HCL output?
    # waiting to see if https://github.com/virtuald/pyhcl
    # will ever support a 'dumps' method. If they do, then
    # it may be worth implementing a to_hcl() method here.