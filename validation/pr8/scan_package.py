from pathlib import Path
import re,json,zipfile,io,hashlib,sys,struct
PAT=re.compile(r'wheeltec|轮趣|MiniBalance|R680A|R550A_PLUS|\bT280\w*|\bS(?:100|150|180|200|260|300)\w*|taobao\.com|tmall\.com',re.I)
private=re.compile(r'/Users/|/home/(?!runner|user)[^/\s]+/|[A-Za-z]:\\(?:Users|Desktop|ZZZ[^\\]*)\\')
results={'files':0,'nested_archives':0,'identity_matches':[],'private_path_matches':[],'uninspected':[]}
def scan(name,b):
 results['files']+=1
 if PAT.search(name):results['identity_matches'].append({'file':name,'where':'filename'})
 if zipfile.is_zipfile(io.BytesIO(b)):
  results['nested_archives']+=1
  with zipfile.ZipFile(io.BytesIO(b)) as z:
   for n in z.namelist():
    if not n.endswith('/'):scan(name+'!'+n,z.read(n))
  return
 if Path(name).suffix.lower() in ['.7z','.rar','.pdf','.png','.jpg','.jpeg']:
  results['uninspected'].append(name)
 if Path(name).suffix.lower()=='.stl' and len(b)>=84 and 84+50*struct.unpack('<I',b[80:84])[0]==len(b):
  b=b[:80]  # Binary STL remainder is numeric geometry, not document text.
 texts=[]
 for enc in ['utf-8','gb18030','utf-16le','utf-16be']:
  try:texts.append(b.decode(enc))
  except UnicodeDecodeError:pass
 for t in texts:
  matches=sorted(set(m.group() for m in PAT.finditer(t)))
  if matches:results['identity_matches'].append({'file':name,'tokens':matches});break
 for t in texts:
  if private.search(t):results['private_path_matches'].append(name);break
for file in sorted(Path(sys.argv[1]).glob('*.zip')):scan(file.name,file.read_bytes())
Path(sys.argv[2]).write_text(json.dumps(results,indent=2)+'\n');print({k:(len(v) if isinstance(v,list) else v) for k,v in results.items()})
if results['identity_matches'] or results['private_path_matches'] or results['uninspected']:sys.exit(1)
