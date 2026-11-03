package com.example.rovercontrol

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import android.util.Base64
import android.util.Log
import android.view.ViewGroup
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.compose.ui.platform.LocalLifecycleOwner
import okhttp3.*
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

private const val TAG = "RoverControl"

// ─── Colors ───
val DarkBg = Color(0xFF1a1a2e)
val CardBg = Color(0xFF16213e)
val HeaderBg = Color(0xFF0f3460)
val AccentGreen = Color(0xFF06d6a0)
val AccentRed = Color(0xFFef476f)
val AccentBlue = Color(0xFF4361ee)
val AccentPurple = Color(0xFF7b2ff7)
val AccentOrange = Color(0xFFff9f1c)
val TextDim = Color(0xFF8d99ae)

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme(
                colorScheme = darkColorScheme(
                    background = DarkBg,
                    surface = CardBg,
                    primary = AccentBlue,
                    secondary = AccentGreen,
                    error = AccentRed
                )
            ) {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = DarkBg
                ) {
                    RoverApp()
                }
            }
        }
    }
}

@Composable
fun RoverApp() {
    var ipAddress by remember { mutableStateOf("192.168.1.100") }
    var selectedModel by remember { mutableStateOf("bag") } // "bag" or "general"
    var isConnected by remember { mutableStateOf(false) }
    var webSocket by remember { mutableStateOf<WebSocket?>(null) }
    var statusMessage by remember { mutableStateOf("") }
    var trackingState by remember { mutableStateOf("IDLE") }

    val client = remember {
        OkHttpClient.Builder()
            .readTimeout(0, TimeUnit.MILLISECONDS)
            .build()
    }

    if (!isConnected) {
        ConnectScreen(
            ipAddress = ipAddress, 
            onIpChange = { ipAddress = it }, 
            selectedModel = selectedModel,
            onModelChange = { selectedModel = it },
            onConnect = {
                val request = Request.Builder().url("ws://$ipAddress:8765").build()
                webSocket = client.newWebSocket(request, object : WebSocketListener() {
                    override fun onOpen(webSocket: WebSocket, response: Response) {
                        isConnected = true
                        // Send model choice immediately
                        val json = JSONObject()
                        json.put("cmd", "set_model")
                        json.put("model", selectedModel)
                        webSocket.send(json.toString())
                    }
                    override fun onMessage(webSocket: WebSocket, text: String) {
                        try {
                            val json = JSONObject(text)
                            when (json.optString("type")) {
                                "matched" -> {
                                    val sim = json.optDouble("similarity", 0.0)
                                    statusMessage = "Target locked! (${(sim * 100).toInt()}% match)"
                                    trackingState = "TRACKING"
                                }
                                "no_match" -> {
                                    statusMessage = json.optString("message", "No match found")
                                    trackingState = "NO_MATCH"
                                }
                            }
                        } catch (e: Exception) {
                            Log.e(TAG, "Message parse error", e)
                        }
                    }
                    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                        isConnected = false
                        Log.e(TAG, "Connection failed", t)
                    }
                    override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                        isConnected = false
                    }
                })
            }
        )
    } else {
        DriveScreen(
            webSocket = webSocket,
            statusMessage = statusMessage,
            trackingState = trackingState,
            onStatusChange = { statusMessage = it },
            onTrackingStateChange = { trackingState = it },
            onDisconnect = {
                webSocket?.close(1000, "User disconnected")
                isConnected = false
                trackingState = "IDLE"
                statusMessage = ""
            }
        )
    }
}

// ─────────────────────────── Connection Screen ───────────────────────────

@Composable
fun ConnectScreen(
    ipAddress: String, 
    onIpChange: (String) -> Unit, 
    selectedModel: String,
    onModelChange: (String) -> Unit,
    onConnect: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(32.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center
    ) {
        Icon(
            Icons.Default.Sensors,
            contentDescription = null,
            tint = AccentBlue,
            modifier = Modifier.size(64.dp)
        )
        Spacer(modifier = Modifier.height(16.dp))
        Text(
            "Rover Control",
            fontSize = 28.sp,
            fontWeight = FontWeight.Bold,
            color = Color.White
        )
        Text(
            "Connect to your laptop station",
            fontSize = 14.sp,
            color = TextDim
        )
        Spacer(modifier = Modifier.height(32.dp))

        OutlinedTextField(
            value = ipAddress,
            onValueChange = onIpChange,
            label = { Text("Laptop IP Address") },
            singleLine = true,
            modifier = Modifier.fillMaxWidth(),
            colors = OutlinedTextFieldDefaults.colors(
                focusedBorderColor = AccentBlue,
                unfocusedBorderColor = TextDim
            )
        )
        Spacer(modifier = Modifier.height(24.dp))

        // Model Selection UI
        Text(
            "Select AI Architecture:",
            fontSize = 16.sp,
            fontWeight = FontWeight.Bold,
            color = Color.White,
            modifier = Modifier.align(Alignment.Start)
        )
        Spacer(modifier = Modifier.height(8.dp))
        
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            RadioButton(
                selected = selectedModel == "bag",
                onClick = { onModelChange("bag") },
                colors = RadioButtonDefaults.colors(selectedColor = AccentBlue)
            )
            Text("Custom Bag Model", color = Color.White)
        }
        
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            RadioButton(
                selected = selectedModel == "general",
                onClick = { onModelChange("general") },
                colors = RadioButtonDefaults.colors(selectedColor = AccentBlue)
            )
            Text("General Model (People/Objects)", color = Color.White)
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            RadioButton(
                selected = selectedModel == "aruco",
                onClick = { onModelChange("aruco") },
                colors = RadioButtonDefaults.colors(selectedColor = AccentBlue)
            )
            Text("ArUco Marker Follower", color = Color.White)
        }

        Spacer(modifier = Modifier.height(32.dp))

        Button(
            onClick = onConnect,
            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp),
            colors = ButtonDefaults.buttonColors(containerColor = AccentBlue),
            shape = RoundedCornerShape(12.dp)
        ) {
            Icon(Icons.Default.Link, contentDescription = null)
            Spacer(modifier = Modifier.width(8.dp))
            Text("Connect", fontSize = 16.sp, fontWeight = FontWeight.Bold)
        }
    }
}

// ─────────────────────────── Drive Screen ───────────────────────────

@Composable
fun DriveScreen(
    webSocket: WebSocket?,
    statusMessage: String,
    trackingState: String,
    onStatusChange: (String) -> Unit,
    onTrackingStateChange: (String) -> Unit,
    onDisconnect: () -> Unit
) {
    val context = LocalContext.current
    var capturedPhotos by remember { mutableStateOf<List<Bitmap>>(emptyList()) }
    var showCaptureMode by remember { mutableStateOf(false) }
    var hasCameraPermission by remember {
        mutableStateOf(
            ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA)
                    == PackageManager.PERMISSION_GRANTED
        )
    }

    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        hasCameraPermission = granted
        if (granted) showCaptureMode = true
        else Toast.makeText(context, "Camera permission required", Toast.LENGTH_SHORT).show()
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // ── Header Bar ──
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = HeaderBg),
            shape = RoundedCornerShape(12.dp)
        ) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 16.dp, vertical = 10.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Circle, contentDescription = null,
                        tint = AccentGreen, modifier = Modifier.size(12.dp))
                    Spacer(modifier = Modifier.width(8.dp))
                    Text("Connected", color = AccentGreen, fontSize = 14.sp,
                        fontWeight = FontWeight.SemiBold)
                }
                IconButton(onClick = onDisconnect) {
                    Icon(Icons.Default.LinkOff, contentDescription = "Disconnect",
                        tint = AccentRed)
                }
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // ── Status Card ──
        val stateColor = when (trackingState) {
            "TRACKING" -> AccentGreen
            "NO_MATCH" -> AccentOrange
            "SEARCHING" -> AccentOrange
            else -> TextDim
        }
        val stateIcon = when (trackingState) {
            "TRACKING" -> Icons.Default.GpsFixed
            "SEARCHING" -> Icons.Default.GpsNotFixed
            else -> Icons.Default.GpsOff
        }

        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = CardBg),
            shape = RoundedCornerShape(10.dp)
        ) {
            Row(
                modifier = Modifier.padding(16.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(stateIcon, contentDescription = null,
                    tint = stateColor, modifier = Modifier.size(28.dp))
                Spacer(modifier = Modifier.width(12.dp))
                Column {
                    Text(
                        trackingState,
                        color = stateColor,
                        fontSize = 18.sp,
                        fontWeight = FontWeight.Bold
                    )
                    if (statusMessage.isNotEmpty()) {
                        Text(statusMessage, color = TextDim, fontSize = 12.sp)
                    }
                }
            }
        }

        Spacer(modifier = Modifier.height(12.dp))

        // ── Capture Target Section ──
        if (showCaptureMode) {
            CaptureTargetCard(
                capturedPhotos = capturedPhotos,
                onCapture = { bitmap ->
                    if(capturedPhotos.size < 3) capturedPhotos = capturedPhotos + bitmap
                },
                onClearPhotos = {
                    capturedPhotos = emptyList()
                },
                onSend = {
                    val photosArray = JSONArray()
                    for (bmp in capturedPhotos) {
                        val stream = ByteArrayOutputStream()
                        bmp.compress(Bitmap.CompressFormat.JPEG, 85, stream)
                        val b64 = Base64.encodeToString(stream.toByteArray(), Base64.NO_WRAP)
                        photosArray.put(b64)
                    }
                    val json = JSONObject()
                    json.put("cmd", "reference_photos")
                    json.put("photos", photosArray)
                    webSocket?.send(json.toString())

                    onStatusChange("Sent ${capturedPhotos.size} photos, matching...")
                    onTrackingStateChange("SEARCHING")
                    showCaptureMode = false
                },
                onCancel = {
                    capturedPhotos = emptyList()
                    showCaptureMode = false
                }
            )
            Spacer(modifier = Modifier.height(12.dp))
        }

        // ── Control Buttons ──
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // Capture Target
            Button(
                onClick = {
                    capturedPhotos = emptyList()
                    if (hasCameraPermission) showCaptureMode = true
                    else permissionLauncher.launch(Manifest.permission.CAMERA)
                },
                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),
                colors = ButtonDefaults.buttonColors(containerColor = AccentPurple),
                shape = RoundedCornerShape(10.dp)
            ) {
                Icon(Icons.Default.CameraAlt, contentDescription = null,
                    modifier = Modifier.size(20.dp))
                Spacer(modifier = Modifier.width(6.dp))
                Text("Select Target", fontSize = 14.sp, fontWeight = FontWeight.Bold)
            }

            // Start Following
            Button(
                onClick = {
                    webSocket?.send(JSONObject().put("cmd", "start").toString())
                    onTrackingStateChange("TRACKING")
                },
                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),
                colors = ButtonDefaults.buttonColors(containerColor = AccentGreen),
                shape = RoundedCornerShape(10.dp)
            ) {
                Icon(Icons.Default.PlayArrow, contentDescription = null,
                    modifier = Modifier.size(20.dp), tint = Color.Black)
                Spacer(modifier = Modifier.width(6.dp))
                Text("Follow", fontSize = 14.sp, fontWeight = FontWeight.Bold,
                    color = Color.Black)
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // Stop
            Button(
                onClick = {
                    webSocket?.send(JSONObject().put("cmd", "stop").toString())
                    onTrackingStateChange("IDLE")
                    onStatusChange("")
                },
                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),
                colors = ButtonDefaults.buttonColors(containerColor = AccentRed),
                shape = RoundedCornerShape(10.dp)
            ) {
                Icon(Icons.Default.Stop, contentDescription = null,
                    modifier = Modifier.size(20.dp))
                Spacer(modifier = Modifier.width(6.dp))
                Text("Stop", fontSize = 14.sp, fontWeight = FontWeight.Bold)
            }

            // Clear Target
            Button(
                onClick = {
                    webSocket?.send(JSONObject().put("cmd", "clear_target").toString())
                    onTrackingStateChange("IDLE")
                    onStatusChange("")
                    capturedPhotos = emptyList()
                },
                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),
                colors = ButtonDefaults.buttonColors(containerColor = HeaderBg),
                shape = RoundedCornerShape(10.dp)
            ) {
                Icon(Icons.Default.Clear, contentDescription = null,
                    modifier = Modifier.size(20.dp), tint = TextDim)
                Spacer(modifier = Modifier.width(6.dp))
                Text("Clear", fontSize = 14.sp, color = TextDim)
            }
        }

        Spacer(modifier = Modifier.weight(1f))

        // ── Joystick ──
        Text("Manual Drive", color = TextDim, fontSize = 12.sp)
        Spacer(modifier = Modifier.height(6.dp))
        Joystick { x, y ->
            val speed = 255
            val left = ((y + x) * speed).toInt().coerceIn(-speed, speed)
            val right = ((y - x) * speed).toInt().coerceIn(-speed, speed)

            val json = JSONObject()
            json.put("cmd", "manual_drive")
            json.put("left", left)
            json.put("right", right)
            webSocket?.send(json.toString())
        }
        Spacer(modifier = Modifier.height(20.dp))
    }
}

// ─────────────────────────── Capture Target Card ───────────────────────────

@Composable
fun CaptureTargetCard(
    capturedPhotos: List<Bitmap>,
    onCapture: (Bitmap) -> Unit,
    onClearPhotos: () -> Unit,
    onSend: () -> Unit,
    onCancel: () -> Unit
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraExecutor = remember { Executors.newSingleThreadExecutor() }
    val imageCapture = remember { ImageCapture.Builder().build() }
    val preview = remember { Preview.Builder().build() }
    
    // Default to back camera for targeting someone else
    var isBackCamera by remember { mutableStateOf(true) }

    // Bind camera function
    fun bindCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(context)
        cameraProviderFuture.addListener({
            val cameraProvider = cameraProviderFuture.get()
            cameraProvider.unbindAll()
            try {
                val selector = if (isBackCamera) CameraSelector.DEFAULT_BACK_CAMERA else CameraSelector.DEFAULT_FRONT_CAMERA
                cameraProvider.bindToLifecycle(
                    lifecycleOwner,
                    selector,
                    preview,
                    imageCapture
                )
            } catch (e: Exception) {
                Log.e(TAG, "Camera bind failed", e)
            }
        }, ContextCompat.getMainExecutor(context))
    }

    LaunchedEffect(isBackCamera) {
        bindCamera()
    }

    DisposableEffect(Unit) {
        onDispose {
            try {
                ProcessCameraProvider.getInstance(context).get().unbindAll()
            } catch (_: Exception) {}
        }
    }

    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = CardBg),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier.padding(12.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text("Select Target for Rover", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 15.sp)
                IconButton(onClick = { isBackCamera = !isBackCamera }) {
                    Icon(Icons.Default.FlipCameraAndroid, contentDescription = "Switch Camera", tint = AccentPurple)
                }
            }

            // Live Camera Preview (shows what the user is about to capture)
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(200.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color.Black)
            ) {
                AndroidView(
                    factory = { ctx ->
                        val previewView = PreviewView(ctx).apply {
                            this.scaleType = PreviewView.ScaleType.FILL_CENTER
                            layoutParams = ViewGroup.LayoutParams(
                                ViewGroup.LayoutParams.MATCH_PARENT,
                                ViewGroup.LayoutParams.MATCH_PARENT
                            )
                        }
                        preview.setSurfaceProvider(previewView.surfaceProvider)
                        previewView
                    },
                    modifier = Modifier.fillMaxSize()
                )

                // Overlays
                if (capturedPhotos.size >= 3) {
                    Box(modifier = Modifier.fillMaxSize().background(Color.Black.copy(alpha=0.6f)), contentAlignment = Alignment.Center) {
                        Text("Ready to Track!", color = Color.White, fontWeight = FontWeight.Bold)
                    }
                } else {
                    Icon(
                        Icons.Default.Add, 
                        contentDescription = null, 
                        tint = Color.White.copy(alpha=0.5f),
                        modifier = Modifier.align(Alignment.Center).size(36.dp)
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(8.dp))
            Text("Snap 1-3 photos of the person's clothing.", color = TextDim, fontSize = 12.sp)
            Spacer(modifier = Modifier.height(8.dp))

            // Photo thumbnails & Clear button
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.Center,
                verticalAlignment = Alignment.CenterVertically
            ) {
                for (i in 0 until 3) {
                    Box(
                        modifier = Modifier
                            .size(60.dp)
                            .padding(4.dp)
                            .clip(RoundedCornerShape(8.dp))
                            .border(
                                1.dp,
                                if (i < capturedPhotos.size) AccentPurple else TextDim.copy(alpha = 0.2f),
                                RoundedCornerShape(8.dp)
                            )
                            .background(HeaderBg),
                        contentAlignment = Alignment.Center
                    ) {
                        if (i < capturedPhotos.size) {
                            Image(
                                bitmap = capturedPhotos[i].asImageBitmap(),
                                contentDescription = "Photo ${i + 1}",
                                modifier = Modifier.fillMaxSize(),
                                contentScale = ContentScale.Crop
                            )
                        } else {
                            Text("${i + 1}", color = TextDim.copy(alpha = 0.4f), fontSize = 16.sp)
                        }
                    }
                }
                
                if (capturedPhotos.isNotEmpty()) {
                    IconButton(onClick = onClearPhotos) {
                        Icon(Icons.Default.Refresh, contentDescription = "Retake", tint = AccentRed)
                    }
                }
            }

            Spacer(modifier = Modifier.height(14.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // Cancel
                OutlinedButton(
                    onClick = onCancel,
                    modifier = Modifier.weight(1f),
                    shape = RoundedCornerShape(8.dp)
                ) {
                    Text("Cancel", fontSize = 12.sp)
                }

                // Snap Photo
                Button(
                    onClick = {
                        if (capturedPhotos.size >= 3) {
                            Toast.makeText(context, "Max 3 photos", Toast.LENGTH_SHORT).show()
                            return@Button
                        }
                        imageCapture.takePicture(
                            cameraExecutor,
                            object : ImageCapture.OnImageCapturedCallback() {
                                override fun onCaptureSuccess(image: ImageProxy) {
                                    val bitmap = image.toBitmap()
                                    val scaled = Bitmap.createScaledBitmap(bitmap, 320, 240, true)
                                    onCapture(scaled)
                                    image.close()
                                }
                                override fun onError(exception: ImageCaptureException) {
                                    Log.e(TAG, "Capture failed", exception)
                                }
                            }
                        )
                    },
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(containerColor = AccentPurple),
                    shape = RoundedCornerShape(8.dp)
                ) {
                    Icon(Icons.Default.PhotoCamera, contentDescription = null,
                        modifier = Modifier.size(16.dp))
                    Spacer(modifier = Modifier.width(4.dp))
                    Text("Snap", fontSize = 12.sp)
                }

                // Send & Track
                Button(
                    onClick = onSend,
                    modifier = Modifier.weight(1f),
                    enabled = capturedPhotos.isNotEmpty(),
                    colors = ButtonDefaults.buttonColors(containerColor = AccentGreen),
                    shape = RoundedCornerShape(8.dp)
                ) {
                    Icon(Icons.Default.Send, contentDescription = null,
                        modifier = Modifier.size(16.dp), tint = Color.Black)
                    Spacer(modifier = Modifier.width(4.dp))
                    Text("Send", fontSize = 12.sp, fontWeight = FontWeight.Bold,
                        color = Color.Black)
                }
            }
        }
    }
}

// ─────────────────────────── Joystick ───────────────────────────

@Composable
fun Joystick(onMove: (Float, Float) -> Unit) {
    var thumbPosition by remember { mutableStateOf(Offset.Zero) }
    val radius = 150f

    Canvas(
        modifier = Modifier
            .size(140.dp)
            .background(HeaderBg, shape = CircleShape)
            .pointerInput(Unit) {
                detectDragGestures(
                    onDragEnd = {
                        thumbPosition = Offset.Zero
                        onMove(0f, 0f)
                    },
                    onDragCancel = {
                        thumbPosition = Offset.Zero
                        onMove(0f, 0f)
                    }
                ) { change, dragAmount ->
                    change.consume()
                    val newPos = thumbPosition + dragAmount
                    val distance = newPos.getDistance()

                    thumbPosition = if (distance <= radius) {
                        newPos
                    } else {
                        newPos * (radius / distance)
                    }

                    val normX = thumbPosition.x / radius
                    val normY = -(thumbPosition.y / radius)
                    onMove(normX, normY)
                }
            }
    ) {
        // Crosshair guides
        drawLine(TextDim.copy(alpha = 0.2f), Offset(center.x, 0f),
            Offset(center.x, size.height), strokeWidth = 1f)
        drawLine(TextDim.copy(alpha = 0.2f), Offset(0f, center.y),
            Offset(size.width, center.y), strokeWidth = 1f)

        // Thumb
        drawCircle(
            color = AccentBlue,
            radius = 35f,
            center = center + thumbPosition
        )
    }
}
