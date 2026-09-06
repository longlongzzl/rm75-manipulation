"""Dependency-light WSGI frontend/API, mountable alongside the existing Flask app."""
from __future__ import annotations
from pathlib import Path
from urllib.parse import urlsplit
from .io import loads,dumps


class WorkcellWSGI:
    def __init__(self,service,fallback=None):
        self.service=service;self.fallback=fallback
        self.static=Path(__file__).resolve().parents[1]/'web/static/workcell'

    def __call__(self,env,start_response):
        path=env.get('PATH_INFO','/');method=env.get('REQUEST_METHOD','GET')
        def response(code,payload,content_type='application/json; charset=utf-8'):
            data=(payload if isinstance(payload,bytes) else dumps(payload).encode())
            start_response(code,[('Content-Type',content_type),('Content-Length',str(len(data))),
                ('Cache-Control','no-store'),('X-Content-Type-Options','nosniff'),
                ('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; frame-ancestors 'none'"),
                ('Referrer-Policy','no-referrer')])
            return [data]
        if path=='/' and self.fallback is None:
            path='/workcell/'
        if path in ('/workcell','/workcell/','/workcell/index.html','/workcell/app.js','/workcell/style.css'):
            if method!='GET':
                return response('405 Method Not Allowed',{'error':'GET only'})
            name=path.rsplit('/',1)[-1]
            if name in ('','workcell'):
                name='index.html'
            types={'index.html':'text/html; charset=utf-8','app.js':'application/javascript; charset=utf-8','style.css':'text/css; charset=utf-8'}
            return response('200 OK',(self.static/name).read_bytes(),types[name])
        if not path.startswith('/api/workcell/'):
            if self.fallback:
                if method not in ('GET','HEAD','OPTIONS') and (self.service.active or self.service.latch.exists()) and not path.endswith('/stop'):
                    return response('409 Conflict',{'error':'Three-scene worker owns the workcell or requires inspection; legacy writes are blocked'})
                if path=='/' and method=='GET':
                    headers=[]
                    def capture(status,items,exc_info=None):
                        headers.extend([status,items])
                    iterable=self.fallback(env,capture)
                    try:
                        body=b''.join(iterable)
                    finally:
                        if hasattr(iterable,'close'):
                            iterable.close()
                    if b'<body>' in body and any(k.lower()=='content-type' and 'text/html' in v for k,v in headers[1]):
                        nav=('<a href="/workcell/" style="position:fixed;right:22px;bottom:22px;z-index:99999;'
                             'padding:12px 18px;background:#176b66;color:white;border-radius:8px;text-decoration:none;'
                             'font:14px system-ui">三场景工作台 →</a>').encode()
                        body=body.replace(b'<body>',b'<body>'+nav,1)
                    final_headers=[(k,v) for k,v in headers[1] if k.lower()!='content-length']
                    final_headers.append(('Content-Length',str(len(body))))
                    start_response(headers[0],final_headers)
                    return [body]
                return self.fallback(env,start_response)
            return response('404 Not Found',{'error':'No route'})
        try:
            if method=='GET' and path=='/api/workcell/info':
                return response('200 OK',self.service.info())
            if method=='GET' and path.startswith('/api/workcell/jobs/'):
                return response('200 OK',self.service.job(path.split('/')[-1]))
            if method!='POST':
                return response('405 Method Not Allowed',{'error':'Unsupported method'})
            origin=env.get('HTTP_ORIGIN')
            host=env.get('HTTP_HOST','')
            if host.split(':')[0] not in ('localhost','127.0.0.1','[::1]'):
                raise PermissionError('Workcell API is restricted to loopback; use SSH tunnelling')
            if origin and urlsplit(origin).netloc!=host:
                raise PermissionError('Cross-origin request refused')
            if env.get('HTTP_X_WORKCELL_TOKEN')!=self.service.csrf:
                raise PermissionError('Missing/incorrect same-session API token')
            size=int(env.get('CONTENT_LENGTH') or 0)
            if not 0<size<=2_000_000:
                raise ValueError('Invalid request size')
            raw=env['wsgi.input'].read(size)
            if len(raw)!=size:
                raise ValueError('Incomplete request body')
            payload=loads(raw.decode())
            if path=='/api/workcell/arm':
                value=self.service.arm(payload['spec'],payload.get('confirmation'))
            elif path=='/api/workcell/jobs':
                value=self.service.submit(payload['spec'],payload.get('arm_token'))
            elif path.endswith('/input') and path.startswith('/api/workcell/jobs/'):
                value=self.service.respond_input(path.split('/')[-2],payload.get('nonce'),payload.get('value'))
            elif path.endswith('/stop') and path.startswith('/api/workcell/jobs/'):
                value=self.service.cancel(path.split('/')[-2])
            else:
                return response('404 Not Found',{'error':'No route'})
            return response('200 OK',value)
        except PermissionError as exc:
            return response('403 Forbidden',{'error':str(exc)})
        except FileNotFoundError as exc:
            return response('404 Not Found',{'error':str(exc)})
        except RuntimeError as exc:
            return response('409 Conflict',{'error':str(exc)})
        except (ValueError,TypeError,KeyError) as exc:
            return response('400 Bad Request',{'error':str(exc)})
        except Exception as exc:
            return response('500 Internal Server Error',{'error':f'{type(exc).__name__}: {exc}'})


def mount(flask_app,profile_path,app_root,*,allow_real=False):
    from .service import WorkcellService
    # Public process-manager attributes used by the existing frontend are scanned
    # conservatively. Separate legacy programs outside this server must be stopped.
    def legacy_busy():
        import sys,subprocess
        module=sys.modules.get('rm75_app.web.control_panel')
        if module is None:
            return False
        seen=set()
        def active(value,depth):
            if id(value) in seen or depth>2:
                return False
            seen.add(id(value))
            if isinstance(value,subprocess.Popen):
                return value.poll() is None
            if isinstance(value,dict):
                return any(active(v,depth+1) for v in value.values())
            if type(value).__module__==module.__name__ and hasattr(value,'__dict__'):
                return active(vars(value),depth+1)
            return False
        return any(active(v,0) for v in vars(module).values())
    service=WorkcellService(app_root,profile_path,allow_real=allow_real,legacy_busy=legacy_busy)
    flask_app.wsgi_app=WorkcellWSGI(service,flask_app.wsgi_app)
    flask_app.extensions['rm75_workcell']=service
    return service
