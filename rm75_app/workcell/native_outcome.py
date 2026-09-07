"""Observe original cycle/final markers; a process return is not task success."""
import re
import threading


class NativeOutcomeCapture:
    def __init__(self, stream):
        self.stream=stream;self.pending='';self.cycles=[];self.final=None;self.clearance_failures=[]
        self.lock=threading.RLock()

    def __getattr__(self, name):return getattr(self.stream,name)

    def write(self,text):
        with self.lock:
            self.stream.write(text);self.pending+=text
            while '\n' in self.pending:
                line,self.pending=self.pending.split('\n',1)
                # Prefetch may print between the main print's text and newline.
                # Require a line-start marker and an exact boolean token.
                match=re.match(r'cycle (\d+) success = (True|False)(?=$|[^A-Za-z0-9_])',line.strip())
                if match:self.cycles.append({'cycle':int(match[1]),'success':match[2]=='True'})
                match=re.match(r'final success = (True|False)(?=$|[^A-Za-z0-9_])',line.strip())
                if match:self.final=match[1]=='True'
                if line.startswith('[warn] post-place clearance planning failed after release;'):
                    self.clearance_failures.append(line)
        return len(text)

    def report(self,expected_cycles):
        cycles=self.cycles
        # Native retries print the same index after a failed attempt. Preserve
        # those failures while allowing the original retry to finish that piece.
        next_cycle=1;ordered=True;completed=[]
        for row in cycles:
            if row['cycle']!=next_cycle:
                ordered=False
            if row['success']:
                completed.append(row['cycle']);next_cycle+=1
        passed=(self.final is True and ordered and next_cycle==expected_cycles+1
                and not self.clearance_failures)
        return {'native_cycles':list(cycles),'native_final_success':self.final,
                'native_completed_cycles':completed,
                'clearance_failures':list(self.clearance_failures),
                'expected_cycles':expected_cycles,'native_full_chain_passed':passed}
