# Filename: gcp_credentials.py
# Created : 15 Aug 2017
# Author  : Derek Ripper
# Purpose : Contains 2 sets of GCP(google cloud platform) credentials using 2
#           independent "google service accounts".
#           Set up to falcilitate use of google "cloud speech recogintion API"
#           Used by s2t.py

def gcp_credentials(keyowner): 
        # NB Derek's creit card expires on 31 July 2017!! so use Zeke's generated key below....
   if keyowner == "Derek" :
	   credentials = r"""{
	  "type": "service_account",
	  "project_id": "gcp-sr",
	  "private_key_id": "aa385b5e2f487b3b6adea1210428934506bef0a1",
	  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDv0cxm0Bdl+qJe\nqh16Jahj4lCLE8S5k38rj8iBKSurn3hYRtOdUfhwBwKXB1PpYoM2VS4L28nSdklt\n09AW1ogiEVa4z8MxRAUySJlRomEtINfZqVkFTLSOtppMtmML+WpktIVmRDJuQmxG\nLmn6LldFINuDWNheAwMmP+hjHiEu2A3ihImc7J2zt4tCTeQNC/39Z9jugegmBQZs\nww4AbXuB37Z71JK+mUvvZPv+h+gg2GSrgJKM9sH3543CZplcvRLseAjPWTDItRpy\nxOPDB9Y6wWVus3P3cB0NGLOSXKNLXFwFthHazrOQKzORiSK/j25XYlipz+Ruhm6k\nC3nb019bAgMBAAECggEATUyOWcVRBWnX1DN49NoWgLt9wpZInphQMTZTJm6iyNrJ\n64pIwzicn19jElMmVN+P839ZLDFXyCKgYGoZdIMJthFopoExJTwLgL2tzYZNVEJ1\n0I6pRvAGcsmgyoEvQ7jM9lDJfsEUkD2QsL5dImq1bm680oVcmFDYPwfyW6BmibYy\nZnHTfgKr5o3MXAoB+C9Y0LV/TQSkSfldTAVTko3i66rZ88gta5mCE5J+oYSbc9nb\nNUGRW2CW//GKWwMw3aRqoF+so3xJzy7+tRXWeC37pjyma5MDYRMs5rkWAFdxig4b\nfwFHDynwtJ3ZHoIZzZmnaBu0yW2/dJtZrlP1zlf8YQKBgQD6PsDywlr91hB7OLe1\n0HlGSvDZOFVfIWuvjGDr4ol6xHYXVEO7niHWcvl8y9qj2agn4W/+Vp46jYGWfiwC\nanjKhXNWAw+k57g7MfAufgN/De5tMxfB0Y46RFi+16Lg8WluCR/dxbrL4fuKkQ63\nK/6GQm4mCaaV0Yh5QYBZh6WJVQKBgQD1Vaq/CsaiOe3+5zy3+lbi786YIxl37D6B\nOjm1oTGLzDmDow5USy/KpQkc8Agx/0h8mBOyvZG/+vI6AsSZ8RZ4VdSAWFJ/q10r\nmfiV4aLoEPJsIqygZ6IE/CiOxIx1ykV3A47PZJ0893NPqCo82Xl7CemFsvzYHDTm\n96J9bDOF7wKBgHYm8PTtnQaibo+vXNXkQ45TzdnRxkUvQ2fUUOKuyBiF7/fd2kkY\nRYO6L1+j5GxeVQ3XXAhrHzQoIdpLYj4VxUhhr+4ZbeZ/XbXdQzjAWKhBjKRUblAd\nwBh0sq4QpB+u/AdvGXOday/eV+S5zoffpsH/VYByKAwurVALBC3BZQAtAoGBAIsS\nsAMyOZ221xpbvQjSCbUFmgiWRRa9PkWFWzeCFBMahzP/F91i7cmjOoJD83FcNJwk\nnW4Cln/M4slNzmMxzroSda734nRrERrpYoicavvAt5vjIBaiCK9ovhkIhFM1gaFQ\nzAD3GUd5Qs3SF3d9FKdR3CYla72aZ8bSdDNDRgXTAoGAH3qx/xg4VEi7Y+47SUOg\nDBJvVv4bmk5pf+JCB3nwz8XANoJLwXTdTMkFiKkxJLowpRSQ7thisbIY1ksbjNm6\n9idr+/pyHPPHpEoEwrrD6H1Fil4p/6gyyGcehHLsnP5AIGAtJoxeTTIJAU227brz\nfjh1Xgv254Dkp5BVdzNOlZI=\n-----END PRIVATE KEY-----\n",
	  "client_email": "derek-752@gcp-sr.iam.gserviceaccount.com",
	  "client_id": "103420411461651475926",
	  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
	  "token_uri": "https://accounts.google.com/o/oauth2/token",
	  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
	  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/derek-752%40gcp-sr.iam.gserviceaccount.com"
	}
	"""
   elif keyowner == "Zeke":
           credentials = r"""{
	  "type": "service_account",
	  "project_id": "avian-lane-171413",
	  "private_key_id": "e30bf1553ee64a01f319543341c7b956ca0e8e87",
	  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQC6wB8tzIwq9ii8\nqeKREdpTcrv8ceo4Z7204WcjxJ0ys3volQ28pRklt5ssi6Qu7QC/6QmDKkaKofCu\n7to4FcwcKsYNdYBpdxgmbTwqin5/w+D656crNHQFgJon0U2M4rjaAXNSNo/yKkpj\nnViT9EHFJZdTrDA5sQ9d99sSSLcyrj17vvgUSLjWgAaXjUgko4gVoE5aWnr3Y3iy\nZ6rnUioCZBqI0Ay7U1d89Ovw321obaAXYVT2XdjdoYStcn3cOE4A6+HzHmRpzHsA\nJUfqw6WFijT2Q8UntocF7NGutQQDZxTGmOeZV4zTjbvsjHSkq3nuTtrEK3JEuKI8\nn90/QbLJAgMBAAECggEAFaoTg+HikdpQJADWSIuIZx1civ7hE82PH6R1bVwR+2+A\nbJ0cBJonhm1cGkkR0SLL7zd208lL+SmrrPOHC7IVpV/d7XJH5RQ8kDqkW/AImERO\nPXUxuqyhWAERARUCGNm1ZY79BJjRTp/S1OcMP0+68Ia5nSqQxqSQeo92EdMvM3j5\nhDyXLxHLIPyXxGho51mJQWy2wtnX18Hjrm5/L7aPyU0vUrUzvVjyKH7mCNMSlF3z\nbZ0zb1+8ccgVryKl4pwMofkwSKRRz/pyxZdEJWgvuApU0FSNBycFpwDofYWWWoYB\n5O6Y4FtQkCUG5gIOoFQeDGcNlBn7txf+x6F4OMev7QKBgQD4JsVD306DMJx+V1Uv\nKQ8KE8B6j+0aLekOU2lgSzQgJHRy5MTJEXcpeVl8jj7iIuB8zaK/oASiJtyWSuy5\n7pLr10kfRdbaFqIZdlnLxdA9ToqIcjeQ4NTQBacvl00ome3SSKv+JD77VA01/jnQ\nnbH0mUFlYH3BVM5F5bHLG6ZdvQKBgQDAqDNcjaL3kpZWxzieEAvW07gsF+t/oJE4\n0f5/d8S5N/e7YE/W01AAvnfZ5jJsQwQ3qnaIL5PrKWeaFoN5lrCyj+/6hOuIdn7d\nNtwAYqkSZOALPyJOZkd4STqUndaxV+9q6s3hlo89XIfWQ0dhnxaZy7WZt15D2Hsn\nmLVSVba7/QKBgCMR56MV9hSYVCWO1h0aImP8MqGfoZSnlF5P736KYk2AmWx7ZEw+\nSab03W6686wl3bAFp9CJHt6Du30KDbahPuZRwKXIyvp7ZGFQ1pPz2uAvL2jlK3Ew\nNSNJCT2yllb8mh2z44rBOJ1wXYWZ7jXLc5Dr79AR+PPPtm8ubRgVSkfJAoGBAKHX\n6yXSOlg+RZtqys5F7pwuyeYIxiY2LikMh/5vD80FDlDTjN6MMAYf+7EKZ8t41P8q\nbV7kkR+ZbdYNGa+3/oJS1qeep/rAiLyvTEvqldvE8E2iYOB5nYqYAeU9X1El7RPl\nYsUi5PwIUniVlk6VBbh3X/xdBYx7PKI77/7V0KfFAoGBAJVGcYbeXtfak5XHG2Y0\nZE6Xw26A25Kz1zm/MSTGzHzqvSLyqympyDEEiZ29w7VoPlb2WVlqZ4cvMOca37VI\n/moAMYxywBT3ojUAaog0ah3dhKX9la8ZhoGw51pwRNvpV7b1fXk0YaNDL3X7/ewT\n0CVvMnO5Z2W3h9IE2vV18EX9\n-----END PRIVATE KEY-----\n",
	  "client_email": "service-account@avian-lane-171413.iam.gserviceaccount.com",
	  "client_id": "101796461215192531281",
	  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
	  "token_uri": "https://accounts.google.com/o/oauth2/token",
	  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
	  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/service-account%40avian-lane-171413.iam.gserviceaccount.com"
	}
        """

   return credentials  
